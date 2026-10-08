use base64::{engine::general_purpose::STANDARD, Engine};
use robomaster::{
    server::{router, AppState, Module},
    worker::{supervise, Snapshot, Worker, WorkerError},
};
use serde_json::json;
use std::{net::Ipv4Addr, path::PathBuf, process::Stdio, sync::Arc, time::Duration};
use tokio::{
    net::TcpListener,
    sync::{mpsc, watch, RwLock},
    task::JoinSet,
};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

const HELP: &str = "RoboMaster 本机仿真启动器

推荐：在仓库根目录执行 ./rm，一次启动导航和自瞄对战。

  robomaster up [--navigation-port N] [--duel-port N] [--root DIR] [--bin-dir DIR]
  robomaster serve navigation|duel [--port N] [--root DIR] [--bin-dir DIR]
  robomaster run navigation|duel|cube [--root DIR] [--bin-dir DIR] [-- 原生模块参数...]

模块：navigation 导航；duel 自瞄对战；cube 物理魔方。
默认地址：导航 http://127.0.0.1:8765/；自瞄对战 http://127.0.0.1:8766/。
up 在同一终端运行两个服务；按 Ctrl+C 同时关闭，并回收本次启动的仿真进程。
端口已占用时停止启动，不关闭已有服务、不自动换端口；可显式指定两个新端口。
所有网页服务仅监听本机 127.0.0.1。

导航单独启动还可指定：--localization prior|slam --power-budget W。
魔方窗口：robomaster run cube -- --viewer；双夹爪模式再加 --dual。
魔方无窗口：robomaster run cube -- --headless --duration 2。
根目录必须包含 assets/，原生程序默认位于该目录的 build/bin/。
未指定 --root 时优先使用 ROBOMASTER_ROOT 环境变量，否则使用编译时的仓库目录。
直接运行本启动器需要已编译的原生程序；./rm 会检查并准备构建。
--root 指定仓库目录；--bin-dir 指定原生程序目录；--help 查看本帮助。";

struct Shutdown {
    #[cfg(unix)]
    interrupt: tokio::signal::unix::Signal,
    #[cfg(unix)]
    terminate: tokio::signal::unix::Signal,
}
impl Shutdown {
    fn new() -> Result<Self> {
        #[cfg(unix)]
        {
            use tokio::signal::unix::{signal, SignalKind};
            Ok(Self {
                interrupt: signal(SignalKind::interrupt())
                    .map_err(|e| format!("无法安装 Ctrl+C 处理器：{e}"))?,
                terminate: signal(SignalKind::terminate())
                    .map_err(|e| format!("无法安装 SIGTERM 处理器：{e}"))?,
            })
        }
        #[cfg(not(unix))]
        Ok(Self {})
    }

    async fn recv(&mut self) {
        #[cfg(unix)]
        tokio::select! {
            _ = self.interrupt.recv() => {},
            _ = self.terminate.recv() => {},
        }
        #[cfg(not(unix))]
        {
            let _ = tokio::signal::ctrl_c().await;
        }
    }
}

#[tokio::main]
async fn main() {
    if let Err(error) = run().await {
        eprintln!("错误：{error}");
        std::process::exit(1);
    }
}

fn option_value(args: &mut impl Iterator<Item = String>, name: &str) -> Result<String> {
    args.next()
        .filter(|value| !value.is_empty() && !value.starts_with("--"))
        .ok_or_else(|| format!("{name} 缺少参数值；请使用 --help 查看用法").into())
}

fn port_value(args: &mut impl Iterator<Item = String>, name: &str) -> Result<u16> {
    let value = option_value(args, name)?;
    value
        .parse::<u16>()
        .ok()
        .filter(|port| *port != 0)
        .ok_or_else(|| format!("{name} 的端口必须是 1–65535 的整数，收到：{value}").into())
}

fn module_title(module: Module) -> &'static str {
    match module {
        Module::Navigation => "导航仿真",
        Module::Duel => "自瞄对战",
        Module::Cube => "物理魔方",
    }
}

struct Service {
    module: Module,
    port: u16,
    binary: PathBuf,
    args: Vec<String>,
}

struct Prepared {
    module: Module,
    port: u16,
    listener: TcpListener,
    worker: Worker,
    snapshot: Snapshot,
}

async fn stop_prepared(prepared: &mut [Prepared]) {
    // 初始化阶段尚未启动后台任务，逐一关闭并等待回收所有已创建的子进程。
    for service in prepared {
        service.worker.stop().await;
    }
}

async fn serve(root: PathBuf, services: Vec<Service>, shutdown: &mut Shutdown) -> Result<()> {
    // 先占用所有端口；任何端口失败都发生在启动第一个仿真进程之前。
    let mut listeners = Vec::new();
    for service in &services {
        let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, service.port))
            .await
            .map_err(|error| {
                format!(
                    "{}端口 {} 无法监听：{error}。未启动仿真；已有服务保持不变。请指定其他端口。",
                    module_title(service.module),
                    service.port
                )
            })?;
        listeners.push(listener);
    }
    let mut prepared = Vec::new();
    for (service, listener) in services.into_iter().zip(listeners) {
        println!("正在初始化{}，请稍候……", module_title(service.module));
        let worker = match Worker::spawn(&service.binary, &root, &service.args).await {
            Ok(worker) => worker,
            Err(error) => {
                stop_prepared(&mut prepared).await;
                return Err(format!("{}启动失败：{error}", module_title(service.module)).into());
            }
        };
        prepared.push(Prepared {
            module: service.module,
            port: service.port,
            listener,
            worker,
            snapshot: Snapshot::default(),
        });
        let current = prepared.last_mut().expect("刚加入的服务必须存在");
        let initialization = async {
            current.snapshot.state = current.worker.call("state", json!({})).await?;
            if current.module == Module::Navigation {
                current.snapshot.map = Some(current.worker.call("map", json!({})).await?);
            }
            // 首帧渲染也属于初始化；显卡或相机失败时不留下另一个半启动的服务。
            for view in current.module.views() {
                let frame = current.worker.call("frame", json!({"view": view})).await?;
                let image = frame
                    .get("data")
                    .and_then(serde_json::Value::as_str)
                    .and_then(|data| STANDARD.decode(data).ok())
                    .filter(|bytes| bytes.starts_with(&[0xff, 0xd8, 0xff]))
                    .ok_or_else(|| {
                        WorkerError::Unavailable("原生程序返回的首帧不是有效 JPEG".into())
                    })?;
                current.snapshot.frames.insert((*view).into(), image);
            }
            Ok::<_, WorkerError>(())
        };
        let result = tokio::select! {
            biased;
            _ = shutdown.recv() => None,
            result = initialization => Some(result),
        };
        match result {
            Some(Ok(())) => {}
            Some(Err(error)) => {
                stop_prepared(&mut prepared).await;
                return Err(format!(
                    "{}初始化失败：{error}；已回收本次启动的仿真进程。",
                    module_title(service.module)
                )
                .into());
            }
            None => {
                stop_prepared(&mut prepared).await;
                println!("已取消启动，仿真进程已回收。");
                return Ok(());
            }
        }
    }

    let (stop, stopped) = watch::channel(false);
    let mut workers = JoinSet::new();
    let mut servers = JoinSet::new();
    for service in prepared {
        let snapshot = Arc::new(RwLock::new(service.snapshot));
        let (commands, receiver) = mpsc::channel(64);
        workers.spawn(supervise(
            service.worker,
            service.module,
            receiver,
            Arc::clone(&snapshot),
            stopped.clone(),
        ));
        let app = router(AppState {
            module: service.module,
            root: root.clone(),
            commands,
            snapshot,
        });
        let mut server_stopped = stopped.clone();
        servers.spawn(async move {
            axum::serve(service.listener, app)
                .with_graceful_shutdown(async move {
                    if !*server_stopped.borrow() {
                        let _ = server_stopped.changed().await;
                    }
                })
                .await
        });
        println!(
            "{}已启动：http://127.0.0.1:{}/",
            module_title(service.module),
            service.port
        );
    }
    println!("请在浏览器打开上述地址；按 Ctrl+C 关闭本次启动的全部服务。");
    let mut failure = None;
    tokio::select! {
        _ = shutdown.recv() => {},
        result = servers.join_next() => {
            failure = match result {
                Some(Ok(Ok(()))) | None => Some("网页服务意外结束".to_owned()),
                Some(Ok(Err(error))) => Some(format!("网页服务失败：{error}")),
                Some(Err(error)) => Some(format!("网页服务任务失败：{error}")),
            };
        }
    }
    let _ = stop.send(true);
    while let Some(result) = workers.join_next().await {
        if let Err(error) = result {
            failure.get_or_insert_with(|| format!("仿真进程回收失败：{error}"));
        }
    }
    // 限制HTTP连接的排空时间，避免未结束的浏览器请求阻止终端退出。
    if tokio::time::timeout(Duration::from_secs(3), async {
        while let Some(result) = servers.join_next().await {
            match result {
                Ok(Ok(())) => {}
                Ok(Err(error)) => {
                    failure.get_or_insert_with(|| format!("网页服务关闭失败：{error}"));
                }
                Err(error) => {
                    failure.get_or_insert_with(|| format!("网页服务回收失败：{error}"));
                }
            }
        }
    })
    .await
    .is_err()
    {
        servers.abort_all();
        while servers.join_next().await.is_some() {}
    }
    println!("本次启动的服务已关闭，仿真进程已回收。");
    if let Some(error) = failure {
        return Err(error.into());
    }
    Ok(())
}

async fn run() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let Some(action) = args.next() else {
        println!("{HELP}");
        return Ok(());
    };
    if matches!(action.as_str(), "-h" | "--help" | "help") {
        println!("{HELP}");
        return Ok(());
    }
    if !matches!(action.as_str(), "up" | "serve" | "run") {
        return Err(format!("未知操作：{action}\n{HELP}").into());
    }
    let module = if action == "up" {
        None
    } else {
        let name = args
            .next()
            .ok_or("缺少模块名：请选择 navigation、duel 或 cube")?;
        if matches!(name.as_str(), "--help" | "-h") {
            println!("{HELP}");
            return Ok(());
        }
        Some(name.parse::<Module>()?)
    };
    if action == "serve" && module == Some(Module::Cube) {
        return Err(
            "物理魔方使用原生窗口：robomaster run cube -- --viewer；双夹爪模式再加 --dual。".into(),
        );
    }
    let mut root = std::env::var_os("ROBOMASTER_ROOT")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../.."));
    let mut bin_dir = None;
    let mut port = module.map(Module::port).unwrap_or(8765);
    let mut navigation_port = 8765;
    let mut duel_port = 8766;
    let mut worker_args = Vec::new();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--root" => root = PathBuf::from(option_value(&mut args, &arg)?),
            "--bin-dir" => bin_dir = Some(PathBuf::from(option_value(&mut args, &arg)?)),
            "--port" if action == "serve" => port = port_value(&mut args, &arg)?,
            "--navigation-port" if action == "up" => navigation_port = port_value(&mut args, &arg)?,
            "--duel-port" if action == "up" => duel_port = port_value(&mut args, &arg)?,
            "--localization" | "--power-budget" if module == Some(Module::Navigation) => {
                let value = option_value(&mut args, &arg)?;
                worker_args.push(arg);
                worker_args.push(value);
            }
            "--" if action == "run" => {
                worker_args.extend(args);
                break;
            }
            "--help" | "-h" => {
                println!("{HELP}");
                return Ok(());
            }
            _ if action == "run" => worker_args.push(arg),
            _ => return Err(format!("未知选项：{arg}；请使用 --help 查看用法").into()),
        }
    }
    root = root
        .canonicalize()
        .map_err(|error| format!("仓库目录无法访问 {}：{error}", root.display()))?;
    if !root.join("assets").is_dir() {
        return Err(format!(
            "仓库目录 {} 缺少 assets/，请用 --root 指向仓库根目录。",
            root.display()
        )
        .into());
    }
    let bin_dir = bin_dir.unwrap_or_else(|| root.join("build/bin"));
    let binary = |module: Module| -> Result<PathBuf> {
        let path = bin_dir.join(format!("rm_{}", module.name()));
        path.canonicalize().map_err(|error| {
            format!("找不到{}原生程序 {}：{error}。请在仓库根目录执行 ./rm build，或用 --bin-dir 指定程序目录。", module_title(module), path.display()).into()
        })
    };
    // 在创建任何子进程前注册信号，初始化期间也可以安全取消。
    let mut shutdown = Shutdown::new()?;
    if action == "run" {
        let module = module.expect("run 已校验模块名");
        let mut child = tokio::process::Command::new(binary(module)?)
            .arg("--root")
            .arg(root)
            .args(worker_args)
            .stdin(Stdio::inherit())
            .stdout(Stdio::inherit())
            .stderr(Stdio::inherit())
            .kill_on_drop(true)
            .spawn()
            .map_err(|error| format!("{}启动失败：{error}", module_title(module)))?;
        let status = tokio::select! {
            status = child.wait() => Some(status.map_err(|error| format!("等待原生进程失败：{error}"))?),
            _ = shutdown.recv() => {
                child.kill().await.map_err(|error| format!("关闭原生进程失败：{error}"))?;
                child.wait().await.map_err(|error| format!("回收原生进程失败：{error}"))?;
                None
            }
        };
        if let Some(status) = status.filter(|status| !status.success()) {
            return Err(format!("{}原生进程异常退出：{status}", module_title(module)).into());
        }
        return Ok(());
    }
    let services = if action == "up" {
        vec![
            Service {
                module: Module::Navigation,
                port: navigation_port,
                binary: binary(Module::Navigation)?,
                args: vec![],
            },
            Service {
                module: Module::Duel,
                port: duel_port,
                binary: binary(Module::Duel)?,
                args: vec![],
            },
        ]
    } else {
        let module = module.expect("serve 已校验模块名");
        vec![Service {
            module,
            port,
            binary: binary(module)?,
            args: worker_args,
        }]
    };
    serve(root, services, &mut shutdown).await
}
