// swift-tools-version:5.9
import PackageDescription

let package = Package(
    name: "WigglerCore",
    platforms: [.iOS(.v17), .macOS(.v13)],
    products: [
        .library(name: "WigglerCore", targets: ["WigglerCore"])
    ],
    targets: [
        .target(name: "WigglerCore"),
        .testTarget(name: "WigglerCoreTests", dependencies: ["WigglerCore"])
    ]
)
