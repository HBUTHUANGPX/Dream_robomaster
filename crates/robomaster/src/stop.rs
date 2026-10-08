use crate::Result;
use std::{
    ffi::OsStr,
    fs, io,
    os::{
        fd::{AsRawFd, FromRawFd, OwnedFd},
        unix::{ffi::OsStrExt, fs::MetadataExt},
    },
    path::{Path, PathBuf},
    time::{Duration, Instant},
};

fn executable_path(path: &Path) -> &Path {
    // 重新构建后，运行中的旧程序仍使用原来的可执行文件。
    let bytes = path.as_os_str().as_bytes();
    Path::new(OsStr::from_bytes(
        bytes.strip_suffix(b" (deleted)").unwrap_or(bytes),
    ))
}

fn belongs_to_repository(process: &Path, root: &Path, executables: &[PathBuf], uid: u32) -> bool {
    let matches = || -> Option<bool> {
        if fs::metadata(process).ok()?.uid() != uid {
            return Some(false);
        }
        let executable = fs::read_link(process.join("exe")).ok()?;
        if !executables
            .iter()
            .any(|path| path == executable_path(&executable))
        {
            return Some(false);
        }
        let command = fs::read(process.join("cmdline")).ok()?;
        let args: Vec<_> = command.split(|byte| *byte == 0).collect();
        match args.get(1).copied()? {
            b"up" => {}
            b"serve" if matches!(args.get(2).copied(), Some(b"navigation" | b"duel")) => {}
            _ => return Some(false),
        }
        // 与启动器一致：最后一个 --root 优先，其次为进程环境和编译目录。
        let declared = args.windows(2).rev().find(|pair| pair[0] == b"--root");
        let directory = if let Some(pair) = declared {
            PathBuf::from(OsStr::from_bytes(pair[1]))
        } else {
            let environment = fs::read(process.join("environ")).ok()?;
            let declared = environment
                .split(|byte| *byte == 0)
                .find_map(|entry| entry.strip_prefix(b"ROBOMASTER_ROOT="));
            if let Some(value) = declared {
                PathBuf::from(OsStr::from_bytes(value))
            } else {
                // 只有同一二进制才能确认编译目录。旧文件可能从其他仓库复制而来。
                let running = fs::metadata(process.join("exe")).ok()?;
                let current = fs::metadata("/proc/self/exe").ok()?;
                if (running.dev(), running.ino()) != (current.dev(), current.ino()) {
                    return Some(false);
                }
                PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
            }
        };
        let directory = if directory.is_absolute() {
            directory
        } else {
            fs::read_link(process.join("cwd")).ok()?.join(directory)
        };
        Some(directory.canonicalize().ok()?.as_path() == root)
    };
    matches().unwrap_or(false)
}

fn exited(process: &OwnedFd) -> io::Result<bool> {
    let mut descriptor = libc::pollfd {
        fd: process.as_raw_fd(),
        events: libc::POLLIN,
        revents: 0,
    };
    // 指针指向一个有效的 pollfd，查询不会等待或转移文件描述符所有权。
    let result = unsafe { libc::poll(&mut descriptor, 1, 0) };
    if result < 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(descriptor.revents & (libc::POLLIN | libc::POLLHUP) != 0)
}

pub async fn services(root: &Path) -> Result<()> {
    let current = std::env::current_exe()?;
    let executables = [
        executable_path(&current).to_path_buf(),
        root.join("target/release/robomaster"),
        root.join("target/debug/robomaster"),
    ];
    // geteuid 不接收指针，也不修改进程状态。
    let uid = unsafe { libc::geteuid() };
    let mut targets = Vec::new();
    for entry in fs::read_dir("/proc").map_err(|error| format!("无法查询本机进程：{error}"))?
    {
        let entry = entry?;
        let Some(pid) = entry
            .file_name()
            .to_str()
            .and_then(|name| name.parse::<i32>().ok())
        else {
            continue;
        };
        if !belongs_to_repository(&entry.path(), root, &executables, uid) {
            continue;
        }
        // pidfd 绑定一个具体进程。即使进程号被复用，也不会向新进程发送信号。
        let descriptor = unsafe { libc::syscall(libc::SYS_pidfd_open, pid, 0) };
        if descriptor < 0 {
            let error = io::Error::last_os_error();
            if error.raw_os_error() == Some(libc::ESRCH) {
                continue;
            }
            return Err(format!(
                "无法确认服务进程 {pid}：{error}。请在启动终端按 Ctrl+C。原进程保持不变。"
            )
            .into());
        }
        // 成功的 pidfd_open 返回一个归调用者所有的新文件描述符。
        let process = unsafe { OwnedFd::from_raw_fd(descriptor as i32) };
        // 打开描述符期间，进程可能退出。重新核对目录和命令后再发信号。
        if !belongs_to_repository(&entry.path(), root, &executables, uid) || exited(&process)? {
            continue;
        }
        targets.push((pid, process));
    }
    if targets.is_empty() {
        println!("当前用户在本仓库没有运行中的网页服务。");
        return Ok(());
    }
    println!("正在停止本仓库的网页服务，并等待仿真进程退出……");
    for (pid, process) in &targets {
        // 描述符保持有效；空指针表示由内核填充信号信息。
        let result = unsafe {
            libc::syscall(
                libc::SYS_pidfd_send_signal,
                process.as_raw_fd(),
                libc::SIGTERM,
                std::ptr::null::<libc::siginfo_t>(),
                0,
            )
        };
        if result < 0 {
            let error = io::Error::last_os_error();
            if error.raw_os_error() != Some(libc::ESRCH) {
                return Err(
                    format!("无法停止服务进程 {pid}：{error}。请在启动终端按 Ctrl+C。").into(),
                );
            }
        }
    }
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let pending: Vec<_> = targets
            .iter()
            .map(|(pid, process)| exited(process).map(|done| (!done).then_some(*pid)))
            .collect::<io::Result<Vec<_>>>()?
            .into_iter()
            .flatten()
            .collect();
        if pending.is_empty() {
            println!("本仓库的网页服务已停止。");
            return Ok(());
        }
        if Instant::now() >= deadline {
            return Err(format!("等待服务退出超时，仍在运行的进程：{pending:?}。请检查启动终端的错误信息，再执行 ./rm stop。").into());
        }
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_different_binary_without_a_declared_root_is_not_assigned_our_build_directory() {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../..")
            .canonicalize()
            .unwrap();
        let directory = root
            .join(".cache/tmp")
            .join(format!("stop-identity-{}", std::process::id()));
        fs::create_dir_all(directory.join("process")).unwrap();
        let executable = directory.join("robomaster");
        fs::write(&executable, "来自另一份源码的程序").unwrap();
        let process = directory.join("process");
        std::os::unix::fs::symlink(&executable, process.join("exe")).unwrap();
        fs::write(process.join("cmdline"), b"robomaster\0up\0").unwrap();
        fs::write(process.join("environ"), b"").unwrap();
        let matched = belongs_to_repository(
            &process,
            &root,
            &[executable],
            fs::metadata(&process).unwrap().uid(),
        );
        fs::remove_dir_all(directory).unwrap();
        assert!(!matched, "没有声明根目录的其他二进制不能视为本仓库服务");
    }
}
