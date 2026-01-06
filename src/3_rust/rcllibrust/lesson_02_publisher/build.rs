fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (source, dependent_paths) =
        roslibrust::codegen::find_and_generate_ros_messages(Vec::new())?;

    let out_dir = std::env::var_os("OUT_DIR").ok_or("OUT_DIR not set")?;
    let dest_path = std::path::Path::new(&out_dir).join("messages.rs");
    std::fs::write(dest_path, source.to_string())?;

    for path in &dependent_paths {
        println!("cargo:rerun-if-changed={}", path.display());
    }

    Ok(())
}
