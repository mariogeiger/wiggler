// swift-tools-version:5.9
import PackageDescription

let package = Package(
    name: "WigglerCore",
    platforms: [.iOS(.v17), .macOS(.v13)],
    products: [
        .library(name: "WigglerCore", targets: ["WigglerCore"])
    ],
    targets: [
        .target(
            name: "WigglerCore",
            // The tracker is CPU-bound; keep the optimiser on in Debug builds too (an unoptimised build is ~20x slower).
            swiftSettings: [.unsafeFlags(["-O"], .when(configuration: .debug))]
        ),
        .testTarget(name: "WigglerCoreTests", dependencies: ["WigglerCore"]),
    ]
)
