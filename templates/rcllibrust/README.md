# rcllibrust template

This is a minimal Rust template intended for your rcllibrust extension.
- The template uses the roslibrust Git repo in `Cargo.toml`.
- Update the dependency to a local path, tag, or commit as needed.
- Common helpers live in `src/qos.rs`, `src/topics.rs`, and `src/services.rs`.
- Adjust the API calls in `src/main.rs` to match your extension.
- This template expects a rosbridge server at `ws://localhost:9090`.
