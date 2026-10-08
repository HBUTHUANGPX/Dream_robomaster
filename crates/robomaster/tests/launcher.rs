#![cfg(unix)]

use std::{
    fs,
    io::{Read, Write},
    net::{Ipv4Addr, TcpListener, TcpStream},
    os::unix::fs::PermissionsExt,
    path::PathBuf,
    process::{Child, Command, Stdio},
    sync::atomic::{AtomicU64, Ordering},
    thread,
    time::{Duration, Instant},
};

static NEXT: AtomicU64 = AtomicU64::new(0);

struct Fixture {
    root: PathBuf,
}
impl Fixture {
    fn new(duel_mode: &str) -> Self {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../.cache/tmp")
            .join(format!(
                "launcher-{}-{}",
                std::process::id(),
                NEXT.fetch_add(1, Ordering::Relaxed)
            ));
        fs::create_dir_all(root.join("assets")).unwrap();
        fs::create_dir_all(root.join("bin")).unwrap();
        let fixture = Self { root };
        fixture.worker("navigation", "normal");
        fixture.worker("duel", duel_mode);
        fixture
    }

    fn worker(&self, module: &str, mode: &str) {
        // 子进程只实现协议，测试真实启动器的端口、HTTP和进程生命周期。
        let response = match mode {
            "normal" => {
                r#"case "$line" in
  *'"command":"frame"'*) printf '%s\n' '{"ok":true,"result":{"data":"/9j/","mime":"image/jpeg"}}' ;;
  *) printf '%s\n' '{"ok":true,"result":{"revision":0,"time":0}}' ;;
esac"#
            }
            "failure" => r#"printf '%s\n' '{"ok":false,"error":"测试初始化失败"}'"#,
            "frame_failure" => {
                r#"case "$line" in
  *'"command":"frame"'*) printf '%s\n' '{"ok":false,"error":"测试图像初始化失败"}' ;;
  *) printf '%s\n' '{"ok":true,"result":{"revision":0,"time":0}}' ;;
esac"#
            }
            "hang" => ":",
            _ => panic!("未知测试模式"),
        };
        let script = format!(
            "#!/bin/sh\nprintf '%s\\n' \"$$\" > \"$2/{module}.pid\"\nwhile IFS= read -r line; do\n{response}\ndone\n"
        );
        let path = self.root.join("bin").join(format!("rm_{module}"));
        fs::write(&path, script).unwrap();
        fs::set_permissions(path, fs::Permissions::from_mode(0o755)).unwrap();
    }

    fn launch(&self, navigation_port: u16, duel_port: u16) -> Running {
        Running {
            child: Command::new(env!("CARGO_BIN_EXE_robomaster"))
                .args(["up", "--root"])
                .arg(&self.root)
                .arg("--bin-dir")
                .arg(self.root.join("bin"))
                .args([
                    "--navigation-port",
                    &navigation_port.to_string(),
                    "--duel-port",
                    &duel_port.to_string(),
                ])
                .stdout(Stdio::piped())
                .stderr(Stdio::piped())
                .spawn()
                .unwrap(),
        }
    }

    fn assert_reaped(&self) {
        for module in ["navigation", "duel"] {
            let path = self.root.join(format!("{module}.pid"));
            if let Ok(pid) = fs::read_to_string(path) {
                assert!(
                    !PathBuf::from(format!("/proc/{}", pid.trim())).exists(),
                    "{module} 的 worker 未被回收: {pid}"
                );
            }
        }
    }

    fn stop(&self) -> std::process::Output {
        Command::new(env!("CARGO_BIN_EXE_robomaster"))
            .args(["stop", "--root"])
            .arg(&self.root)
            .output()
            .unwrap()
    }
}
impl Drop for Fixture {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.root);
    }
}

struct Running {
    child: Child,
}
impl Running {
    fn signal(&self, signal: &str) {
        assert!(Command::new("kill")
            .args([signal, &self.child.id().to_string()])
            .status()
            .unwrap()
            .success());
    }

    fn finish(&mut self) -> (bool, String, String) {
        let deadline = Instant::now() + Duration::from_secs(6);
        let status = loop {
            if let Some(status) = self.child.try_wait().unwrap() {
                break status;
            }
            assert!(Instant::now() < deadline, "启动器没有及时关闭");
            thread::sleep(Duration::from_millis(20));
        };
        let mut stdout = String::new();
        let mut stderr = String::new();
        self.child
            .stdout
            .take()
            .unwrap()
            .read_to_string(&mut stdout)
            .unwrap();
        self.child
            .stderr
            .take()
            .unwrap()
            .read_to_string(&mut stderr)
            .unwrap();
        (status.success(), stdout, stderr)
    }
}
impl Drop for Running {
    fn drop(&mut self) {
        if self.child.try_wait().ok().flatten().is_none() {
            let _ = Command::new("kill")
                .args(["-TERM", &self.child.id().to_string()])
                .status();
            let deadline = Instant::now() + Duration::from_secs(3);
            while Instant::now() < deadline {
                if self.child.try_wait().ok().flatten().is_some() {
                    return;
                }
                thread::sleep(Duration::from_millis(20));
            }
            let _ = self.child.kill();
            let _ = self.child.wait();
        }
    }
}

fn port() -> TcpListener {
    TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap()
}
fn health(port: u16, module: &str) -> bool {
    let Ok(mut stream) = TcpStream::connect_timeout(
        &format!("127.0.0.1:{port}").parse().unwrap(),
        Duration::from_millis(100),
    ) else {
        return false;
    };
    stream
        .set_read_timeout(Some(Duration::from_millis(100)))
        .unwrap();
    let request =
        format!("GET /health HTTP/1.1\r\nHost: localhost:{port}\r\nConnection: close\r\n\r\n");
    if stream.write_all(request.as_bytes()).is_err() {
        return false;
    }
    let mut response = String::new();
    stream.read_to_string(&mut response).is_ok()
        && response.starts_with("HTTP/1.1 200")
        && response.contains(&format!("\"module\":\"{module}\""))
}
fn wait_for(mut predicate: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(4);
    while !predicate() {
        assert!(Instant::now() < deadline, "等待启动条件超时");
        thread::sleep(Duration::from_millis(20));
    }
}

#[test]
fn up_serves_both_modules_and_signals_reap_every_worker() {
    for signal in ["-INT", "-TERM"] {
        let fixture = Fixture::new("normal");
        let first = port();
        let second = port();
        let navigation = first.local_addr().unwrap().port();
        let duel = second.local_addr().unwrap().port();
        drop((first, second));
        let mut running = fixture.launch(navigation, duel);
        wait_for(|| health(navigation, "navigation") && health(duel, "duel"));
        running.signal(signal);
        let (success, stdout, stderr) = running.finish();
        assert!(success, "{stderr}");
        assert!(
            stdout.contains("导航") && stdout.contains("自瞄"),
            "{stdout}"
        );
        assert!(stdout.contains(&format!("http://127.0.0.1:{navigation}/")));
        assert!(stdout.contains(&format!("http://127.0.0.1:{duel}/")));
        fixture.assert_reaped();
        assert!(TcpListener::bind((Ipv4Addr::LOCALHOST, navigation)).is_ok());
        assert!(TcpListener::bind((Ipv4Addr::LOCALHOST, duel)).is_ok());
    }
}

#[test]
fn stop_reaps_both_workers_releases_custom_ports_and_can_repeat() {
    let fixture = Fixture::new("normal");
    let first = port();
    let second = port();
    let navigation = first.local_addr().unwrap().port();
    let duel = second.local_addr().unwrap().port();
    drop((first, second));
    let mut running = fixture.launch(navigation, duel);
    wait_for(|| health(navigation, "navigation") && health(duel, "duel"));
    // 停止只依赖已构建的启动器，不依赖资产或开发环境。
    fs::remove_dir(fixture.root.join("assets")).unwrap();
    let stopped = fixture.stop();
    assert!(
        stopped.status.success(),
        "{}",
        String::from_utf8_lossy(&stopped.stderr)
    );
    assert!(String::from_utf8_lossy(&stopped.stdout).contains("已停止"));
    fixture.assert_reaped();
    assert!(running.finish().0);
    assert!(TcpListener::bind((Ipv4Addr::LOCALHOST, navigation)).is_ok());
    assert!(TcpListener::bind((Ipv4Addr::LOCALHOST, duel)).is_ok());
    let repeated = fixture.stop();
    assert!(repeated.status.success());
    assert!(String::from_utf8_lossy(&repeated.stdout).contains("没有运行"));
}

#[test]
fn stop_preserves_another_repository_and_ignores_stale_pid_files() {
    let fixture = Fixture::new("normal");
    let other = Fixture::new("normal");
    let ports: Vec<_> = (0..4).map(|_| port()).collect();
    let values: Vec<_> = ports
        .iter()
        .map(|p| p.local_addr().unwrap().port())
        .collect();
    drop(ports);
    let mut running = fixture.launch(values[0], values[1]);
    let mut preserved = other.launch(values[2], values[3]);
    wait_for(|| health(values[0], "navigation") && health(values[2], "navigation"));
    fs::create_dir_all(fixture.root.join("output/run")).unwrap();
    fs::write(
        fixture.root.join("output/run/navigation.pid"),
        preserved.child.id().to_string(),
    )
    .unwrap();
    let stopped = fixture.stop();
    assert!(
        stopped.status.success(),
        "{}",
        String::from_utf8_lossy(&stopped.stderr)
    );
    assert!(running.finish().0);
    fixture.assert_reaped();
    assert!(health(values[2], "navigation") && health(values[3], "duel"));
    preserved.signal("-TERM");
    assert!(preserved.finish().0);
    other.assert_reaped();
}

#[test]
fn stop_during_initialization_reaps_workers() {
    let fixture = Fixture::new("hang");
    let first = port();
    let second = port();
    let navigation = first.local_addr().unwrap().port();
    let duel = second.local_addr().unwrap().port();
    drop((first, second));
    let mut running = fixture.launch(navigation, duel);
    wait_for(|| fixture.root.join("duel.pid").exists());
    let stopped = fixture.stop();
    assert!(
        stopped.status.success(),
        "{}",
        String::from_utf8_lossy(&stopped.stderr)
    );
    assert!(running.finish().0);
    fixture.assert_reaped();
}

#[test]
fn root_stop_closes_individual_services_even_after_the_binary_is_replaced() {
    let fixture = Fixture::new("normal");
    // 根入口和运行中的旧二进制来自同一仓库；不需要构建工具或 PID 文件。
    let repository = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    fs::copy(repository.join("rm"), fixture.root.join("rm")).unwrap();
    let executable = fixture.root.join("target/release/robomaster");
    fs::create_dir_all(executable.parent().unwrap()).unwrap();
    fs::hard_link(env!("CARGO_BIN_EXE_robomaster"), &executable).unwrap();
    std::os::unix::fs::symlink(&fixture.root, fixture.root.join("仓库 别名")).unwrap();
    let first = port();
    let second = port();
    let navigation = first.local_addr().unwrap().port();
    let duel = second.local_addr().unwrap().port();
    drop((first, second));
    let mut running = Vec::new();
    for (module, port) in [("navigation", navigation), ("duel", duel)] {
        let mut command = Command::new(&executable);
        command.current_dir(&fixture.root).args(["serve", module]);
        if module == "navigation" {
            command.args(["--root", "仓库 别名"]);
            // 显式根目录优先于环境变量。
            command.env("ROBOMASTER_ROOT", "/不存在的仓库");
        } else {
            command.env("ROBOMASTER_ROOT", "仓库 别名");
        }
        running.push(Running {
            child: command
                .arg("--bin-dir")
                .arg(fixture.root.join("bin"))
                .args(["--port", &port.to_string()])
                .stdout(Stdio::piped())
                .stderr(Stdio::piped())
                .spawn()
                .unwrap(),
        });
    }
    wait_for(|| health(navigation, "navigation") && health(duel, "duel"));
    fs::remove_file(&executable).unwrap();
    fs::copy(env!("CARGO_BIN_EXE_robomaster"), &executable).unwrap();
    for service in &running {
        let old = fs::read_link(format!("/proc/{}/exe", service.child.id())).unwrap();
        assert!(old.to_string_lossy().ends_with(" (deleted)"));
    }
    let stopped = Command::new("bash")
        .arg(fixture.root.join("rm"))
        .arg("stop")
        .current_dir("/")
        .output()
        .unwrap();
    assert!(
        stopped.status.success(),
        "{}",
        String::from_utf8_lossy(&stopped.stderr)
    );
    assert!(String::from_utf8_lossy(&stopped.stdout).contains("已停止"));
    fixture.assert_reaped();
    for service in &mut running {
        assert!(service.finish().0);
    }
    assert!(TcpListener::bind((Ipv4Addr::LOCALHOST, navigation)).is_ok());
    assert!(TcpListener::bind((Ipv4Addr::LOCALHOST, duel)).is_ok());
}

#[test]
fn second_port_conflict_starts_no_workers_and_preserves_existing_listener() {
    let fixture = Fixture::new("normal");
    let first = port();
    let occupied = port();
    let navigation = first.local_addr().unwrap().port();
    let duel = occupied.local_addr().unwrap().port();
    drop(first);
    let (success, _, stderr) = fixture.launch(navigation, duel).finish();
    assert!(!success);
    assert!(
        stderr.contains("端口") && stderr.contains(&duel.to_string()),
        "{stderr}"
    );
    assert!(!fixture.root.join("navigation.pid").exists());
    assert!(!fixture.root.join("duel.pid").exists());
    assert!(TcpListener::bind((Ipv4Addr::LOCALHOST, navigation)).is_ok());
    assert!(occupied.local_addr().is_ok());
}

#[test]
fn second_initialization_failure_reaps_both_workers() {
    let fixture = Fixture::new("failure");
    let first = port();
    let second = port();
    let navigation = first.local_addr().unwrap().port();
    let duel = second.local_addr().unwrap().port();
    drop((first, second));
    let (success, _, stderr) = fixture.launch(navigation, duel).finish();
    assert!(!success && stderr.contains("测试初始化失败"), "{stderr}");
    assert!(fixture.root.join("navigation.pid").exists());
    assert!(fixture.root.join("duel.pid").exists());
    fixture.assert_reaped();
}

#[test]
fn image_initialization_failure_does_not_leave_a_partial_service() {
    let fixture = Fixture::new("frame_failure");
    let first = port();
    let second = port();
    let navigation = first.local_addr().unwrap().port();
    let duel = second.local_addr().unwrap().port();
    drop((first, second));
    let (success, stdout, stderr) = fixture.launch(navigation, duel).finish();
    assert!(
        !success && stderr.contains("测试图像初始化失败"),
        "{stderr}"
    );
    assert!(!stdout.contains("已启动"), "{stdout}");
    fixture.assert_reaped();
}

#[test]
fn signal_during_initialization_reaps_already_started_workers() {
    let fixture = Fixture::new("hang");
    let first = port();
    let second = port();
    let navigation = first.local_addr().unwrap().port();
    let duel = second.local_addr().unwrap().port();
    drop((first, second));
    let mut running = fixture.launch(navigation, duel);
    wait_for(|| fixture.root.join("duel.pid").exists());
    running.signal("-TERM");
    let (success, _, stderr) = running.finish();
    assert!(success, "{stderr}");
    fixture.assert_reaped();
}

#[test]
fn help_and_cli_errors_explain_usage_in_chinese() {
    let help = Command::new(env!("CARGO_BIN_EXE_robomaster"))
        .arg("--help")
        .output()
        .unwrap();
    let text = String::from_utf8(help.stdout).unwrap();
    assert!(text.contains("up") && text.contains("导航") && text.contains("8766"));
    for args in [
        vec!["up", "--duel-port", "0"],
        vec!["serve", "unknown"],
        vec!["up", "--navigation-port"],
        vec!["stop", "navigation"],
        vec!["stop", "--port", "8765"],
    ] {
        let output = Command::new(env!("CARGO_BIN_EXE_robomaster"))
            .args(args)
            .output()
            .unwrap();
        assert!(!output.status.success());
        assert!(String::from_utf8(output.stderr).unwrap().contains("错误"));
    }
}
