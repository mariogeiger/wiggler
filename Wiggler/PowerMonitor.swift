import Foundation
import UIKit

/// Instantaneous battery power draw. iOS has no public API for it; development builds can read the battery gauge
/// (IOPMPowerSource: InstantAmperage in mA, Voltage in mV) through IOKit, which is what this does. When that is not
/// available the value falls back to a coarse estimate from the 1 % battery-level steps.
final class PowerMonitor: ObservableObject {
    @Published private(set) var watts: Double?
    @Published private(set) var isEstimate = false

    private var timer: Timer?
    private var ioLoaded = false
    private var getMatchingService: (@convention(c) (UInt32, CFDictionary?) -> UInt32)?
    private var serviceMatching: (@convention(c) (UnsafePointer<CChar>) -> Unmanaged<CFMutableDictionary>?)?
    private var createProperties: (@convention(c) (UInt32, UnsafeMutablePointer<Unmanaged<CFMutableDictionary>?>, CFAllocator?, UInt32) -> Int32)?
    private var objectRelease: (@convention(c) (UInt32) -> Int32)?
    private var levelSamples: [(Date, Float)] = []
    /// Full-charge energy, iPhone 15 Pro ≈ 3274 mAh × 3.86 V.
    private let batteryWh = 12.6

    func start() {
        UIDevice.current.isBatteryMonitoringEnabled = true
        loadIOKit()
        timer = Timer.scheduledTimer(withTimeInterval: 2, repeats: true) { [weak self] _ in self?.sample() }
        sample()
    }

    func stop() { timer?.invalidate(); timer = nil }

    private func loadIOKit() {
        guard let h = dlopen("/System/Library/Frameworks/IOKit.framework/IOKit", RTLD_NOW) else { return }
        guard let a = dlsym(h, "IOServiceGetMatchingService"), let b = dlsym(h, "IOServiceMatching"),
              let c = dlsym(h, "IORegistryEntryCreateCFProperties"), let d = dlsym(h, "IOObjectRelease") else { return }
        getMatchingService = unsafeBitCast(a, to: (@convention(c) (UInt32, CFDictionary?) -> UInt32).self)
        serviceMatching = unsafeBitCast(b, to: (@convention(c) (UnsafePointer<CChar>) -> Unmanaged<CFMutableDictionary>?).self)
        createProperties = unsafeBitCast(c, to: (@convention(c) (UInt32, UnsafeMutablePointer<Unmanaged<CFMutableDictionary>?>, CFAllocator?, UInt32) -> Int32).self)
        objectRelease = unsafeBitCast(d, to: (@convention(c) (UInt32) -> Int32).self)
        ioLoaded = true
    }

    private func gaugeWatts() -> Double? {
        guard ioLoaded, let matching = serviceMatching?("IOPMPowerSource") else { return nil }
        // IOServiceGetMatchingService consumes the matching dictionary reference returned (+1) by IOServiceMatching.
        let service = getMatchingService?(0, matching.takeUnretainedValue()) ?? 0
        guard service != 0 else { return nil }
        defer { _ = objectRelease?(service) }
        var props: Unmanaged<CFMutableDictionary>?
        guard createProperties?(service, &props, kCFAllocatorDefault, 0) == 0, let dict = props?.takeRetainedValue() as? [String: Any] else {
            return nil
        }
        let amperage = (dict["InstantAmperage"] as? NSNumber)?.doubleValue ?? (dict["Amperage"] as? NSNumber)?.doubleValue
        let voltage = (dict["Voltage"] as? NSNumber)?.doubleValue
        guard let mA = amperage, let mV = voltage, mV > 0 else { return nil }
        // Some gauges report unsigned 64-bit two's complement for negative currents.
        var i = mA
        if i > 1e9 { i -= 18446744073709551616.0 }
        return abs(i) * mV / 1_000_000
    }

    private func sample() {
        if let w = gaugeWatts() {
            watts = w
            isEstimate = false
            return
        }
        // Fallback: energy per 1 % level step.
        let level = UIDevice.current.batteryLevel
        guard level >= 0 else { watts = nil; return }
        let now = Date()
        if levelSamples.isEmpty || levelSamples.last!.1 != level { levelSamples.append((now, level)) }
        if levelSamples.count > 6 { levelSamples.removeFirst() }
        guard levelSamples.count >= 2 else { watts = nil; return }
        let (t0, l0) = levelSamples.first!, (t1, l1) = levelSamples.last!
        let hours = t1.timeIntervalSince(t0) / 3600
        guard hours > 0, l0 > l1 else { watts = nil; return }
        watts = Double(l0 - l1) * batteryWh / hours
        isEstimate = true
    }
}
