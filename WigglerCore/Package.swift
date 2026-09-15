// swift-tools-version:5.9
import PackageDescription

let package = Package(
    name: "WigglerCore",
    platforms: [.iOS(.v17), .macOS(.v13)],
    products: [
        .library(name: "WigglerCore", targets: ["WigglerCore"]),
        .executable(name: "wigreplay", targets: ["wigreplay"]),
    ],
    targets: [
        .target(
            name: "WigglerCore",
            // The tracker is CPU-bound; keep the optimiser on in Debug builds too (an unoptimised build is ~20x slower).
            swiftSettings: [.unsafeFlags(["-O"], .when(configuration: .debug))]
        ),
        .systemLibrary(name: "CZlib", path: "Sources/CZlib"),
        // Replays a .wig recording through the engine and prints the decision journal the app does not record.
        .executableTarget(
            name: "wigreplay", dependencies: ["WigglerCore", "CZlib"],
            swiftSettings: [.unsafeFlags(["-O"], .when(configuration: .debug))]),
        .testTarget(name: "WigglerCoreTests", dependencies: ["WigglerCore"]),
    ]
)
