/// A single fit whose history belongs to one signal, pixel grid and stable angle reference.
public struct HarmonicFit {
    public private(set) var signal: HarmonicSignal
    public private(set) var map: HarmonicMap?
    public let orders: [Int]

    public init(signal: HarmonicSignal = .luma, orders: [Int] = [1, 2, 3]) {
        self.signal = signal
        self.orders = orders
    }

    public var turnProgress: Double { map?.turnProgress ?? 0 }

    public mutating func select(_ signal: HarmonicSignal) {
        guard self.signal != signal else { return }
        self.signal = signal
        map = nil
    }

    public mutating func update(frame: FrameInput?, output: EngineOutput) {
        guard output.axisStable, output.state == .locked else {
            map = nil
            return
        }
        let observation =
            frame.flatMap { signal.image(in: $0) }
            ?? map.map { GrayImage(width: $0.width, height: $0.height, fill: .nan) }
        guard let image = observation else { return }
        if map?.width != image.width || map?.height != image.height {
            map = HarmonicMap(width: image.width, height: image.height, orders: orders)
        }
        map?.update(image: image, output: output)
    }
}
