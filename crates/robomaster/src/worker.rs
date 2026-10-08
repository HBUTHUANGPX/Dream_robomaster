use crate::server::Module;
use base64::{engine::general_purpose::STANDARD, Engine};
use serde_json::{json, Value};
use std::{
    collections::BTreeMap,
    path::Path,
    process::Stdio,
    sync::Arc,
    time::{Duration, Instant},
};
use tokio::{
    io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader},
    process::{Child, ChildStdin, ChildStdout, Command},
    sync::{mpsc, oneshot, watch, RwLock},
};

#[derive(Debug, Clone)]
pub enum WorkerError {
    Command(String),
    Unavailable(String),
}
impl std::fmt::Display for WorkerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Command(e) | Self::Unavailable(e) => f.write_str(e),
        }
    }
}
impl std::error::Error for WorkerError {}

pub struct WorkerRequest {
    pub command: String,
    pub args: Value,
    pub created: Instant,
    pub reply: oneshot::Sender<Result<Value, WorkerError>>,
}

#[derive(Clone)]
pub struct Snapshot {
    pub state: Value,
    pub map: Option<Value>,
    pub frames: BTreeMap<String, Vec<u8>>,
    pub error: Option<String>,
}
impl Default for Snapshot {
    fn default() -> Self {
        Self {
            state: json!({}),
            map: None,
            frames: BTreeMap::new(),
            error: None,
        }
    }
}

pub struct Worker {
    child: Child,
    input: Option<ChildStdin>,
    output: BufReader<ChildStdout>,
}
impl Worker {
    pub async fn spawn(binary: &Path, root: &Path, args: &[String]) -> Result<Self, WorkerError> {
        let mut child = Command::new(binary)
            .arg("--root")
            .arg(root)
            .arg("--rpc")
            .args(args)
            .current_dir(root)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .kill_on_drop(true)
            .spawn()
            .map_err(|e| {
                WorkerError::Unavailable(format!("Cannot start {}: {e}", binary.display()))
            })?;
        let input = child
            .stdin
            .take()
            .ok_or_else(|| WorkerError::Unavailable("No worker stdin".into()))?;
        let output = BufReader::new(
            child
                .stdout
                .take()
                .ok_or_else(|| WorkerError::Unavailable("No worker stdout".into()))?,
        );
        Ok(Self {
            child,
            input: Some(input),
            output,
        })
    }
    pub async fn call(&mut self, command: &str, args: Value) -> Result<Value, WorkerError> {
        let result = tokio::time::timeout(Duration::from_secs(60), async {
            let mut request = serde_json::to_vec(&json!({"command":command,"args":args}))
                .expect("JSON value serializable");
            request.push(b'\n');
            let input = self
                .input
                .as_mut()
                .ok_or_else(|| WorkerError::Unavailable("Worker stopped".into()))?;
            input
                .write_all(&request)
                .await
                .map_err(|e| WorkerError::Unavailable(format!("Worker write failed: {e}")))?;
            input
                .flush()
                .await
                .map_err(|e| WorkerError::Unavailable(e.to_string()))?;
            let mut line = Vec::new();
            (&mut self.output)
                .take(32 * 1024 * 1024 + 1)
                .read_until(b'\n', &mut line)
                .await
                .map_err(|e| WorkerError::Unavailable(format!("Worker read failed: {e}")))?;
            if line.len() > 32 * 1024 * 1024 {
                return Err(WorkerError::Unavailable(
                    "Worker response exceeds 32 MiB".into(),
                ));
            }
            decode_response(&line)
        })
        .await;
        result.unwrap_or_else(|_| {
            Err(WorkerError::Unavailable(format!(
                "Worker {command} timed out after 60 seconds"
            )))
        })
    }
    pub async fn stop(&mut self) {
        // Closing stdin lets a worker at the RPC boundary exit normally.
        drop(self.input.take());
        if tokio::time::timeout(Duration::from_secs(2), self.child.wait())
            .await
            .is_err()
        {
            let _ = self.child.kill().await;
            let _ = self.child.wait().await;
        }
    }
}

fn decode_response(line: &[u8]) -> Result<Value, WorkerError> {
    if line.is_empty() {
        return Err(WorkerError::Unavailable(
            "Native worker exited (EOF)".into(),
        ));
    }
    let value: Value = serde_json::from_slice(line)
        .map_err(|e| WorkerError::Unavailable(format!("Invalid worker JSON: {e}")))?;
    match value.get("ok").and_then(Value::as_bool) {
        Some(true) => value
            .get("result")
            .cloned()
            .ok_or_else(|| WorkerError::Unavailable("Worker result missing".into())),
        Some(false) => Err(WorkerError::Command(
            value
                .get("error")
                .and_then(Value::as_str)
                .unwrap_or("Native command failed")
                .into(),
        )),
        None => Err(WorkerError::Unavailable(
            "Worker response missing ok".into(),
        )),
    }
}

pub async fn supervise(
    mut worker: Worker,
    module: Module,
    requests: mpsc::Receiver<WorkerRequest>,
    snapshot: Arc<RwLock<Snapshot>>,
    mut shutdown: watch::Receiver<bool>,
) {
    // Cancel even an in-flight native call on shutdown. Its stream is discarded,
    // then the child is reaped; a canceled request is never retried on that stream.
    let failure = tokio::select! {
        biased;
        _=shutdown.changed()=>None,
        failure=supervise_loop(&mut worker,module,requests,&snapshot)=>failure,
    };
    if let Some(failure) = failure {
        eprintln!("{} simulation stopped: {failure}", module.name());
        snapshot.write().await.error = Some(failure);
    }
    worker.stop().await;
}

async fn supervise_loop(
    worker: &mut Worker,
    module: Module,
    mut requests: mpsc::Receiver<WorkerRequest>,
    snapshot: &Arc<RwLock<Snapshot>>,
) -> Option<String> {
    let mut tick = tokio::time::interval(module.period());
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let mut frame_count = 0u64;
    loop {
        tokio::select! {
            biased;
            request=requests.recv()=>{
                let Some(mut request)=request else {break None;};
                let limit=Duration::from_millis(if request.command=="drive" {350} else {4000});
                if request.reply.is_closed() || request.created.elapsed()>limit {
                    let _=request.reply.send(Err(WorkerError::Command("Command expired; retry".into())));
                    continue;
                }
                if request.command=="drive" {
                    request.args["_drive_ttl_ms"]=json!(limit.saturating_sub(request.created.elapsed()).as_millis() as u64);
                }
                let result=worker.call(&request.command,request.args).await;
                let fatal=match &result {Err(WorkerError::Unavailable(e))=>Some(e.clone()),_=>None};
                let _=request.reply.send(result);
                if fatal.is_some() {break fatal;}
                match worker.call("state",json!({})).await {
                    Ok(state)=>snapshot.write().await.state=state,
                    Err(e)=>break Some(e.to_string()),
                }
            }
            _=tick.tick()=>{
                let begin=Instant::now();
                match worker.call("tick",json!({})).await {
                    Ok(mut state)=>{state["compute_ms"]=json!((begin.elapsed().as_secs_f64()*10000.0).round()/10.0);snapshot.write().await.state=state;},
                    Err(e)=>break Some(e.to_string()),
                }
                if frame_count%3==0 {
                    let mut error=None;
                    for view in module.views() {
                        match worker.call("frame",json!({"view":view})).await {
                            Ok(frame)=>{
                                match frame.get("data").and_then(Value::as_str).and_then(|s|STANDARD.decode(s).ok()) {
                                    Some(bytes) if bytes.starts_with(&[0xff,0xd8,0xff])=>{snapshot.write().await.frames.insert((*view).into(),bytes);},
                                    _=>{error=Some("Worker returned invalid JPEG".into());break;},
                                }
                            }
                            Err(e)=>{error=Some(e.to_string());break;},
                        }
                    }
                    if error.is_some() {break error;}
                }
                frame_count+=1;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn protocol_distinguishes_validation_failure_from_dead_worker() {
        assert_eq!(
            decode_response(br#"{"ok":true,"result":{"time":1}}"#).unwrap(),
            json!({"time":1})
        );
        assert!(matches!(
            decode_response(br#"{"ok":false,"error":"out of range"}"#),
            Err(WorkerError::Command(_))
        ));
        for bytes in [
            &b""[..],
            &b"log text\n"[..],
            &b"{}"[..],
            &b"{\"ok\":true}"[..],
        ] {
            assert!(matches!(
                decode_response(bytes),
                Err(WorkerError::Unavailable(_))
            ));
        }
    }
    #[tokio::test]
    async fn shutdown_interrupts_a_worker_that_never_responds() {
        let mut child = Command::new("sleep")
            .arg("30")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .kill_on_drop(true)
            .spawn()
            .unwrap();
        let worker = Worker {
            input: child.stdin.take(),
            output: BufReader::new(child.stdout.take().unwrap()),
            child,
        };
        let (_commands, receiver) = mpsc::channel(1);
        let snapshot = Arc::new(RwLock::new(Snapshot::default()));
        let (stop, stopped) = watch::channel(false);
        let task = tokio::spawn(supervise(worker, Module::Duel, receiver, snapshot, stopped));
        tokio::time::sleep(Duration::from_millis(50)).await;
        stop.send(true).unwrap();
        tokio::time::timeout(Duration::from_secs(3), task)
            .await
            .expect("worker must stop within 3 seconds")
            .unwrap();
    }
}
