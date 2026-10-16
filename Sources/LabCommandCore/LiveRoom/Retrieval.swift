import Foundation

/// One topic's `RequestSync` progress (protocol §5). Only a `SyncEnd` carrying the outstanding
/// token moves it to `receiving`, and only receipt of what that manifest names completes it: no
/// timer, request flag or silence does. A timeout sends a fresh token if the need remains.
struct Retrieval: Sendable, Equatable {
    enum Phase: Sendable, Equatable {
        case idle
        /// A request is out; no `SyncEnd` for its token yet.
        case waiting
        /// The `SyncEnd` arrived: the answer is complete once what it delivered is held.
        case receiving(RoomMemory.SyncEnd)
    }

    var phase = Phase.idle
    /// The outstanding request's token; dead once the phase returns to idle.
    var token: RoomMemory.RequestToken?
    /// The generation the request expected.
    var generation: UInt64?
    var sentAt: TimeInterval = 0
    /// Sends of the current need, 1 for the first.
    var attempt = 0
    /// The last send on this topic, for the one-per-interval limit.
    var lastSentAt: TimeInterval?

    var isOutstanding: Bool { phase != .idle }

    /// True when `token` answers the outstanding request.
    func matches(_ token: RoomMemory.RequestToken) -> Bool {
        isOutstanding && self.token == token
    }

    /// Back to idle; the token dies, the send limit stays.
    mutating func reset() {
        phase = .idle
        token = nil
        generation = nil
        attempt = 0
    }

    /// Waiting or receiving for longer than this attempt's timeout.
    func timedOut(at now: TimeInterval) -> Bool {
        guard isOutstanding, attempt > 0 else { return false }
        let timeouts = RoomMemoryMirror.requestRetry
        return now - sentAt >= timeouts[Swift.min(attempt - 1, timeouts.count - 1)]
    }

    /// The per-topic send limit allows a send now.
    func maySend(at now: TimeInterval) -> Bool {
        lastSentAt.map { now - $0 >= RoomMemoryMirror.requestMinInterval } ?? true
    }
}

/// SplitMix64 for request tokens. Seeded from the system generator in production, so two Macs (or
/// two launches) never share a token sequence; tests inject a seed so the reducer stays pure.
struct TokenRandom: RandomNumberGenerator, Sendable, Equatable {
    private var state: UInt64

    init(seed: UInt64) {
        state = seed
    }

    mutating func next() -> UInt64 {
        state &+= 0x9E37_79B9_7F4A_7C15
        var z = state
        z = (z ^ (z >> 30)) &* 0xBF58_476D_1CE4_E5B9
        z = (z ^ (z >> 27)) &* 0x94D0_49BB_1331_11EB
        return z ^ (z >> 31)
    }
}
