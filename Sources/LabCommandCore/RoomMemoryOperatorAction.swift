import Foundation

// MARK: - OperatorAction (protocol §3.6, D46)

extension RoomMemory {
    /// One operator action, Mac → Spark, as the call payload of the node's
    /// `/room_memory/operator/action` service (D46). `eventId` and `expectedState` are both set
    /// (an event action) or both nil (`EVENT_NONE` and `STATE_NONE` on the wire).
    public struct OperatorAction: Sendable, Equatable {
        /// `OperatorAction.ACTION_*`.
        public enum Action: UInt8, Sendable, CaseIterable {
            case acknowledge = 0
            case accept = 1
            case dismiss = 2
            case label = 3
            case markGroundTruth = 4
            case pause = 5
            case resume = 6
            case rereference = 7
            case confirm = 8

            /// Targets one event: `eventId` is required.
            var isEventAction: Bool {
                switch self {
                case .acknowledge, .accept, .dismiss, .label, .confirm: return true
                case .markGroundTruth, .pause, .resume, .rereference: return false
                }
            }

            /// Targets the room: `eventId` must be nil.
            var isRoomAction: Bool { self == .pause || self == .resume || self == .rereference }
        }

        /// `OperatorAction.EVENT_NONE`, the wire value of a missing `event_id`.
        public static let eventNone = UInt64.max
        /// `OperatorAction.STATE_NONE`, the wire value of a missing `expected_state`.
        public static let stateNone: UInt8 = 255
        public static let textMaxBytes = 256
        public static let operatorIdMaxBytes = 64

        /// The common fields: `mapEpoch` is the epoch of the map the operator was shown, `seq`
        /// this client's action counter, `stamp` when the operator acted (never `{0, 0}`).
        public var header: Header
        public var action: Action
        public var eventId: UInt64?
        /// The state the operator saw.
        public var expectedState: ChangeEvent.State?
        /// The generation the operator saw; the node refuses any other (never 0).
        public var expectedGeneration: UInt64
        /// A note, or the label / description of LABEL / MARK_GROUND_TRUTH. "" is none.
        public var text: String
        /// Self-asserted, not authenticated; 1–64 bytes.
        public var operatorId: String
        /// A lowercase UUID; a retry repeats it, so the node acts once.
        public var actionId: String

        public init(header: Header, action: Action, eventId: UInt64?, expectedState: ChangeEvent.State?,
                    expectedGeneration: UInt64, text: String = "", operatorId: String, actionId: String) {
            self.header = header
            self.action = action
            self.eventId = eventId
            self.expectedState = expectedState
            self.expectedGeneration = expectedGeneration
            self.text = text
            self.operatorId = operatorId
            self.actionId = actionId
        }
    }

    /// The `OperatorAction` service's response: what the node did with the action (D46). The
    /// effect itself arrives on the wire as a `ChangeEvent` or map message.
    public enum OperatorOutcome: UInt8, Sendable, CaseIterable {
        case applied = 0
        case invalid = 1
        case wrongRoom = 2
        case staleGeneration = 3
        case staleEpoch = 4
        case unknownEvent = 5
        case stateMismatch = 6
        case notTracking = 7
        case unsupported = 8
        case storageFailed = 9
        case targetChanged = 10
        case rigUnsettled = 11

        /// The response payload: CDR with its encapsulation header, then `uint8 outcome`. Nil when
        /// it is not that, or names no outcome this Mac knows.
        public static func decodeResponse(_ payload: Data) -> OperatorOutcome? {
            let bytes = [UInt8](payload)
            guard bytes.count >= 5, bytes.count <= 5 + RoomCDR.maxTrailingPadding, bytes[0] == 0, bytes[1] <= 1 else {
                return nil
            }
            return OperatorOutcome(rawValue: bytes[4])
        }
    }
}

extension RoomMemory.OperatorAction {
    /// Encodes the action as CDR little endian, including the encapsulation header.
    ///
    /// - Throws: `RoomMemoryCDRError.invalid` when the action breaks the protocol contract (D16).
    public func encodedCDR() throws -> Data {
        try validate()
        var writer = RoomCDRWriter()
        writer.header(header)
        writer.u8(action.rawValue)
        writer.u64(eventId ?? Self.eventNone)
        writer.u8(expectedState?.rawValue ?? Self.stateNone)
        writer.u64(expectedGeneration)
        writer.string(text)
        writer.string(operatorId)
        writer.string(actionId)
        return writer.data
    }

    /// Decodes a CDR `labcommand_msgs/OperatorAction`, including the encapsulation header.
    /// Accepts either byte order.
    ///
    /// - Throws: `RoomMemoryCDRError` on truncated or invalid input.
    public static func decode(cdr data: Data) throws -> RoomMemory.OperatorAction {
        try RoomCDR.decode(data, message: "OperatorAction") { reader in
            let header = try reader.header("")
            let actionValue = try reader.u8("action")
            guard let action = Action(rawValue: actionValue) else {
                throw RoomMemoryCDRError.invalid("OperatorAction: action \(actionValue) is not an ACTION_* value")
            }
            let event = try reader.u64("event_id")
            let stateValue = try reader.u8("expected_state")
            var state: RoomMemory.ChangeEvent.State?
            if stateValue != stateNone {
                guard let known = RoomMemory.ChangeEvent.State(rawValue: stateValue) else {
                    throw RoomMemoryCDRError.invalid("OperatorAction: expected_state \(stateValue) is not a STATE_* value")
                }
                state = known
            }
            let generation = try reader.u64("expected_generation")
            let value = RoomMemory.OperatorAction(
                header: header, action: action, eventId: event == eventNone ? nil : event, expectedState: state,
                expectedGeneration: generation, text: try reader.string("text"),
                operatorId: try reader.string("operator_id"), actionId: try reader.string("action_id")
            )
            try value.validate()
            return value
        }
    }

    func validate() throws {
        let context = "OperatorAction"
        try header.validate(context)
        if header.stamp == RoomMemory.Time(sec: 0, nanosec: 0) {
            throw RoomMemoryCDRError.invalid("\(context): stamp {0, 0} is invalid here (a mark needs its time)")
        }
        if eventId == Self.eventNone {
            throw RoomMemoryCDRError.invalid("\(context): event_id \(Self.eventNone) is EVENT_NONE; use nil")
        }
        guard (eventId == nil) == (expectedState == nil) else {
            throw RoomMemoryCDRError.invalid("\(context): event_id and expected_state are both set or both EVENT_NONE / STATE_NONE")
        }
        if action.isEventAction, eventId == nil {
            throw RoomMemoryCDRError.invalid("\(context): \(action) needs an event_id")
        }
        if action.isRoomAction, eventId != nil {
            throw RoomMemoryCDRError.invalid("\(context): \(action) is room-level; event_id must be EVENT_NONE")
        }
        if let expectedState {
            let allowed: Set<RoomMemory.ChangeEvent.State>?
            switch action {
            case .confirm: allowed = [.candidate]
            case .accept: allowed = [.confirmed]
            case .dismiss: allowed = [.candidate, .confirmed]
            default: allowed = nil
            }
            if let allowed, !allowed.contains(expectedState) {
                throw RoomMemoryCDRError.invalid("\(context): \(action) cannot expect state \(expectedState.rawValue)")
            }
        }
        if expectedGeneration == RoomMemory.generationUnknown {
            throw RoomMemoryCDRError.invalid("\(context): expected_generation 0 is GENERATION_UNKNOWN")
        }
        try Self.requireText(text, 0...Self.textMaxBytes, "\(context): text")
        if action == .label || action == .markGroundTruth, text.isEmpty {
            throw RoomMemoryCDRError.invalid("\(context): \(action) needs text")
        }
        try Self.requireText(operatorId, 1...Self.operatorIdMaxBytes, "\(context): operator_id")
        guard Self.isLowercaseUUID(actionId) else {
            throw RoomMemoryCDRError.invalid("\(context): action_id is not a lowercase UUID (36 characters)")
        }
    }

    private static func requireText(_ text: String, _ bytes: ClosedRange<Int>, _ what: String) throws {
        let count = text.utf8.count
        guard bytes.contains(count) else {
            throw RoomMemoryCDRError.invalid("\(what) is \(count) bytes, outside \(bytes.lowerBound)...\(bytes.upperBound)")
        }
        if text.utf8.contains(0) {
            throw RoomMemoryCDRError.invalid("\(what) contains a NUL")
        }
    }

    /// 8-4-4-4-12 lowercase hex digits.
    static func isLowercaseUUID(_ text: String) -> Bool {
        let bytes = Array(text.utf8)
        guard bytes.count == 36 else { return false }
        for (index, byte) in bytes.enumerated() {
            if [8, 13, 18, 23].contains(index) {
                if byte != UInt8(ascii: "-") { return false }
            } else if !((UInt8(ascii: "0")...UInt8(ascii: "9")).contains(byte) || (UInt8(ascii: "a")...UInt8(ascii: "f")).contains(byte)) {
                return false
            }
        }
        return true
    }
}
