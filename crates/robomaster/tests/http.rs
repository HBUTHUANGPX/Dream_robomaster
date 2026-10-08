use axum::{
    body::Body,
    http::{Request, StatusCode},
};
use http_body_util::BodyExt;
use robomaster::{
    server::{local_request_allowed, router, AppState, Module},
    worker::{Snapshot, WorkerRequest},
};
use serde_json::json;
use std::{path::PathBuf, sync::Arc};
use tokio::sync::{mpsc, RwLock};
use tower::ServiceExt;

fn app() -> (axum::Router, mpsc::Receiver<WorkerRequest>) {
    let (commands, receiver) = mpsc::channel(2);
    let state = AppState {
        module: Module::Duel,
        root: PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../.."),
        commands,
        snapshot: Arc::new(RwLock::new(Snapshot {
            state: json!({"time":1.0,"revision":2}),
            ..Default::default()
        })),
    };
    (router(state), receiver)
}

#[test]
fn forwarded_localhost_and_ipv6_ports_are_accepted() {
    assert!(local_request_allowed(
        Module::Duel,
        "localhost:54232",
        Some("http://localhost:54232")
    ));
    assert!(local_request_allowed(
        Module::Duel,
        "[::1]:8766",
        Some("http://[::1]:8766")
    ));
    assert!(local_request_allowed(
        Module::Navigation,
        "127.0.0.1:8765",
        Some("http://localhost:50995")
    ));
    assert!(local_request_allowed(Module::Duel, "127.0.0.2:80", None));
}

#[test]
fn remote_and_malformed_origins_are_rejected() {
    for (host, origin) in [
        ("localhost:1", "http://evil.example"),
        ("localhost:1", "http://localhost:2"),
        ("localhost:1", "http://user@localhost:1"),
        ("localhost:1", "http://localhost:1/path"),
        ("localhost:1", "http://localhost:1/?x=1"),
        ("localhost:1", "null"),
        ("localhost.evil:1", "http://localhost.evil:1"),
        ("0.0.0.0:1", "http://0.0.0.0:1"),
        ("localhost:0", "http://localhost:0"),
    ] {
        assert!(
            !local_request_allowed(Module::Duel, host, Some(origin)),
            "{host} {origin}"
        );
    }
}

#[tokio::test]
async fn state_snapshot_preserves_revision() {
    let (app, _) = app();
    let response = app
        .oneshot(
            Request::builder()
                .uri("/api/state")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(response.status(), StatusCode::OK);
    let bytes = response.into_body().collect().await.unwrap().to_bytes();
    let value: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
    assert_eq!(value["revision"], 2);
    assert_eq!(value["error"], serde_json::Value::Null);
}

#[tokio::test]
async fn malformed_requests_do_not_enter_command_queue() {
    for (body, content_type, status) in [
        ("[]", "application/json", StatusCode::BAD_REQUEST),
        ("{\"vx\":NaN}", "application/json", StatusCode::BAD_REQUEST),
        ("{}", "text/plain", StatusCode::UNSUPPORTED_MEDIA_TYPE),
    ] {
        let (app, mut receiver) = app();
        let response = app
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/api/drive")
                    .header("host", "localhost:54232")
                    .header("origin", "http://localhost:54232")
                    .header("content-type", content_type)
                    .body(Body::from(body))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), status);
        assert!(receiver.try_recv().is_err());
    }
}

#[tokio::test]
async fn command_response_is_worker_acknowledgement() {
    let (app, mut receiver) = app();
    tokio::spawn(async move {
        let request = receiver.recv().await.unwrap();
        assert_eq!(request.command, "pause");
        assert_eq!(request.args, json!({"paused":true}));
        request
            .reply
            .send(Ok(json!({"revision":3,"paused":true})))
            .unwrap();
    });
    let response = app
        .oneshot(
            Request::builder()
                .method("POST")
                .uri("/api/pause")
                .header("host", "localhost:54232")
                .header("origin", "http://localhost:54232")
                .header("content-type", "application/json")
                .body(Body::from("{\"paused\":true}"))
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(response.status(), StatusCode::OK);
    let bytes = response.into_body().collect().await.unwrap().to_bytes();
    assert_eq!(
        serde_json::from_slice::<serde_json::Value>(&bytes).unwrap()["revision"],
        3
    );
}

#[tokio::test]
async fn oversized_requests_are_rejected_and_missing_frames_are_unavailable() {
    let (app, mut receiver) = app();
    let response = app
        .clone()
        .oneshot(
            Request::builder()
                .method("POST")
                .uri("/api/drive")
                .header("host", "localhost:1")
                .header("content-type", "application/json")
                .body(Body::from(" ".repeat(5000)))
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(response.status(), StatusCode::PAYLOAD_TOO_LARGE);
    assert!(receiver.try_recv().is_err());
    let frame = app
        .oneshot(
            Request::builder()
                .uri("/api/scene.jpg")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(frame.status(), StatusCode::SERVICE_UNAVAILABLE);
}
