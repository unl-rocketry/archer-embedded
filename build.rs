use cargo_metadata::MetadataCommand;

fn main() {
    let metadata = MetadataCommand::new().no_deps().exec().unwrap();
    let protocol_version = metadata.root_package().unwrap().metadata["ProtocolVersion"]
        .as_str()
        .expect("ProtocolVersion must be a string in the format of X.X.X");
    println!("cargo:rustc-env=PROTOCOL_VERSION={}", protocol_version);
    println!("cargo:rustc-link-arg=-Tlinkall.x");
}
