use axum::{
    body::Bytes,
    extract::{rejection::BytesRejection, DefaultBodyLimit, Path, State},
    http::{header, HeaderMap, StatusCode},
    response::{IntoResponse, Response},
    routing::get,
    Router,
};
use serde_json::{json, Value};
use std::{net::IpAddr, path::PathBuf, str::FromStr, sync::Arc, time::Duration};
use tokio::sync::{mpsc, oneshot, RwLock};

use crate::worker::{Snapshot, WorkerError, WorkerRequest};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Module {
    Navigation,
    Duel,
    Cube,
}
impl Module {
    pub fn name(self) -> &'static str {
        match self {
            Self::Navigation => "navigation",
            Self::Duel => "duel",
            Self::Cube => "cube",
        }
    }
    pub fn port(self) -> u16 {
        match self {
            Self::Navigation => 8765,
            Self::Duel => 8766,
            Self::Cube => 8767,
        }
    }
    pub fn period(self) -> Duration {
        Duration::from_millis(if self == Self::Navigation { 100 } else { 50 })
    }
    pub fn views(self) -> &'static [&'static str] {
        match self {
            Self::Duel => &["scene", "camera", "raw", "number"],
            _ => &["scene"],
        }
    }
    fn accepts(self, command: &str) -> bool {
        match self {
            Self::Navigation => matches!(command, "goal" | "pause" | "reset" | "yaw"),
            Self::Duel => matches!(
                command,
                "settings" | "drive" | "aim" | "fire" | "pause" | "reset"
            ),
            Self::Cube => matches!(
                command,
                "scramble" | "solve" | "move" | "gripper" | "pause" | "reset"
            ),
        }
    }
}
impl FromStr for Module {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "navigation" => Ok(Self::Navigation),
            "duel" => Ok(Self::Duel),
            "cube" => Ok(Self::Cube),
            _ => Err(format!(
                "未知模块 {s}；请选择 navigation（导航）、duel（自瞄对战）或 cube（物理魔方）"
            )),
        }
    }
}

#[derive(Clone)]
pub struct AppState {
    pub module: Module,
    pub root: PathBuf,
    pub commands: mpsc::Sender<WorkerRequest>,
    pub snapshot: Arc<RwLock<Snapshot>>,
}

fn loopback_authority(authority: &str) -> bool {
    if authority.is_empty() || authority.contains(['/', '?', '#', '@', '\\']) {
        return false;
    }
    let Ok(parsed) = authority.parse::<axum::http::uri::Authority>() else {
        return false;
    };
    let host = parsed.host();
    let suffix = &authority[host.len()..];
    if !suffix.is_empty()
        && (!suffix.starts_with(':') || suffix[1..].parse::<u16>().map_or(true, |p| p == 0))
    {
        return false;
    }
    if host.eq_ignore_ascii_case("localhost") {
        true
    } else {
        host.trim_matches(['[', ']'])
            .parse::<IpAddr>()
            .is_ok_and(|ip| ip.is_loopback())
    }
}

pub fn local_request_allowed(module: Module, host: &str, origin: Option<&str>) -> bool {
    if !loopback_authority(host) {
        return false;
    }
    let Some(origin) = origin else {
        return true;
    };
    let Some((scheme, authority)) = origin.split_once("://") else {
        return false;
    };
    if !matches!(scheme, "http" | "https") || !loopback_authority(authority) {
        return false;
    }
    module == Module::Navigation || authority.eq_ignore_ascii_case(host)
}

fn response(status: StatusCode, mime: &str, data: impl Into<axum::body::Body>) -> Response {
    (
        status,
        [
            (header::CONTENT_TYPE, mime),
            (header::CACHE_CONTROL, "no-store"),
            (header::X_CONTENT_TYPE_OPTIONS, "nosniff"),
        ],
        data.into(),
    )
        .into_response()
}
fn json_response(status: StatusCode, value: Value) -> Response {
    response(status, "application/json; charset=utf-8", value.to_string())
}
fn error(status: StatusCode, message: impl Into<String>) -> Response {
    let message = message.into();
    json_response(
        status,
        json!({"ok":false,"error":message,"message":message}),
    )
}

pub fn router(state: AppState) -> Router {
    Router::new()
        .route("/", get(index))
        .route("/app.js", get(javascript))
        .route("/health", get(health))
        .route("/api/{endpoint}", get(get_api).post(post_api))
        .layer(DefaultBodyLimit::max(4096))
        .with_state(state)
}

async fn index(State(state): State<AppState>) -> Response {
    let path = state
        .root
        .join("web")
        .join(state.module.name())
        .join("index.html");
    match std::fs::read_to_string(path) {
        Ok(mut html) => {
            if state.module == Module::Navigation {
                if let Some(map) = &state.snapshot.read().await.map {
                    html = html.replacen(
                        "<script>",
                        &format!("<script>window.NAV_INITIAL_MAP={map}</script><script>"),
                        1,
                    );
                }
            }
            response(StatusCode::OK, "text/html; charset=utf-8", html)
        }
        Err(e) => error(StatusCode::NOT_FOUND, format!("UI unavailable: {e}")),
    }
}
async fn javascript(State(state): State<AppState>) -> Response {
    match std::fs::read(
        state
            .root
            .join("web")
            .join(state.module.name())
            .join("app.js"),
    ) {
        Ok(bytes) => response(StatusCode::OK, "text/javascript; charset=utf-8", bytes),
        Err(_) => error(StatusCode::NOT_FOUND, "not found"),
    }
}
async fn health(State(state): State<AppState>) -> Response {
    let snapshot = state.snapshot.read().await;
    json_response(
        if snapshot.error.is_some() {
            StatusCode::SERVICE_UNAVAILABLE
        } else {
            StatusCode::OK
        },
        json!({"module":state.module.name(),"runtime":"C++/Rust","error":snapshot.error}),
    )
}
async fn get_api(State(state): State<AppState>, Path(endpoint): Path<String>) -> Response {
    let snapshot = state.snapshot.read().await;
    match endpoint.as_str() {
        "state" => {
            let mut value = snapshot.state.clone();
            value["error"] = json!(snapshot.error);
            json_response(StatusCode::OK, value)
        }
        "map" if state.module == Module::Navigation => snapshot.map.clone().map_or_else(
            || error(StatusCode::SERVICE_UNAVAILABLE, "Map not ready"),
            |map| json_response(StatusCode::OK, map),
        ),
        "frame.jpg" | "scene.jpg" | "camera.jpg" | "raw.jpg" | "number.jpg" => {
            let view = if endpoint == "frame.jpg" {
                "scene"
            } else {
                endpoint.trim_end_matches(".jpg")
            };
            if !state.module.views().contains(&view) {
                return error(StatusCode::NOT_FOUND, "Unknown view");
            }
            snapshot.frames.get(view).map_or_else(
                || {
                    error(
                        StatusCode::SERVICE_UNAVAILABLE,
                        snapshot.error.as_deref().unwrap_or("Frame not ready"),
                    )
                },
                |frame| response(StatusCode::OK, "image/jpeg", frame.clone()),
            )
        }
        _ => error(StatusCode::NOT_FOUND, "not found"),
    }
}

async fn post_api(
    State(state): State<AppState>,
    Path(command): Path<String>,
    headers: HeaderMap,
    body: Result<Bytes, BytesRejection>,
) -> Response {
    let host = headers
        .get(header::HOST)
        .and_then(|s| s.to_str().ok())
        .unwrap_or("");
    let origin = match headers.get(header::ORIGIN) {
        Some(value) => match value.to_str() {
            Ok(v) => Some(v),
            Err(_) => return error(StatusCode::FORBIDDEN, "Origin rejected"),
        },
        None => None,
    };
    if !local_request_allowed(state.module, host, origin) {
        return error(StatusCode::FORBIDDEN, "Host or Origin rejected");
    }
    let content_type = headers
        .get(header::CONTENT_TYPE)
        .and_then(|h| h.to_str().ok())
        .unwrap_or("")
        .split(';')
        .next()
        .unwrap_or("")
        .trim();
    if content_type != "application/json" {
        return error(StatusCode::UNSUPPORTED_MEDIA_TYPE, "JSON required");
    }
    if !state.module.accepts(&command) {
        return error(StatusCode::NOT_FOUND, "Unknown command");
    }
    let body = match body {
        Ok(body) => body,
        Err(e) => return error(e.status(), e.body_text()),
    };
    let mut args: Value = match serde_json::from_slice(&body) {
        Ok(Value::Object(args)) => Value::Object(args),
        _ => {
            return error(
                StatusCode::BAD_REQUEST,
                "Arguments must be a finite JSON object",
            )
        }
    };
    // The remaining watchdog lifetime is transport metadata, never client input.
    args.as_object_mut().unwrap().remove("_drive_ttl_ms");
    if let Some(reason) = &state.snapshot.read().await.error {
        return error(StatusCode::SERVICE_UNAVAILABLE, reason);
    }
    let (reply, receiver) = oneshot::channel();
    let request = WorkerRequest {
        command,
        args,
        created: std::time::Instant::now(),
        reply,
    };
    if state.commands.try_send(request).is_err() {
        return error(
            StatusCode::SERVICE_UNAVAILABLE,
            "Simulation command queue unavailable",
        );
    }
    match tokio::time::timeout(Duration::from_secs(5), receiver).await {
        Ok(Ok(Ok(value))) => json_response(StatusCode::OK, value),
        Ok(Ok(Err(WorkerError::Command(message)))) => error(StatusCode::BAD_REQUEST, message),
        Ok(Ok(Err(WorkerError::Unavailable(message)))) => {
            error(StatusCode::SERVICE_UNAVAILABLE, message)
        }
        _ => error(
            StatusCode::SERVICE_UNAVAILABLE,
            "Simulation response timed out",
        ),
    }
}
