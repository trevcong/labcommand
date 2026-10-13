import Foundation

/// Session presets already named in RESEARCH-PLAN / REVIEW (A / A2 / B / C).
public enum CapturePreset: String, Sendable, Equatable, CaseIterable, Identifiable {
    case baseline
    case baselineRewalk
    case furnitureMoved
    case yawStress

    public var id: String { rawValue }

    public var code: String {
        switch self {
        case .baseline: return "A"
        case .baselineRewalk: return "A2"
        case .furnitureMoved: return "B"
        case .yawStress: return "C"
        }
    }

    public var layoutTag: String {
        switch self {
        case .baseline: return "baseline"
        case .baselineRewalk: return "baseline_rewalk"
        case .furnitureMoved: return "furniture_moved"
        case .yawStress: return "yaw_stress"
        }
    }

    public var label: String {
        switch self {
        case .baseline: return "A  baseline"
        case .baselineRewalk: return "A2 rewalk"
        case .furnitureMoved: return "B  furniture"
        case .yawStress: return "C  yaw"
        }
    }

    /// Operator-facing title. Codes (A / A2) stay in `code` / `label` for filenames.
    public var operatorTitle: String {
        switch self {
        case .baseline: return "First walk"
        case .baselineRewalk: return "Same path again"
        case .furnitureMoved: return "Walk after a change"
        case .yawStress: return "Turn in place"
        }
    }

    public var operatorDetail: String {
        switch self {
        case .baseline: return "One loop around the basement. About two minutes. This is the baseline bag."
        case .baselineRewalk: return "The same loop, same heading, later. This is the revisit pair for A."
        case .furnitureMoved: return "Same path after you move one thing (chair, door, box)."
        case .yawStress: return "Stand in 2–3 spots and turn 0° / 45° / 90° / 180°."
        }
    }

    /// Record-page card title: research code + what to walk.
    public var deskTitle: String { "\(code)  \(operatorTitle)" }

    public var pathScript: String {
        switch self {
        case .baseline: return "fixed loop path, 90–180 s"
        case .baselineRewalk: return "same path (noise floor)"
        case .furnitureMoved: return "same path; one deliberate change"
        case .yawStress: return "stand in 2–3 places; rotate 0/45/90/180"
        }
    }
}

/// What the operator is about to record. Duration 0 = open (Ctrl-C / Stop).
public struct CaptureSessionSpec: Sendable, Equatable {
    public var preset: CapturePreset
    public var durationSeconds: Int
    public var notes: String
    public var site: String
    public var operatorName: String

    public init(
        preset: CapturePreset = .baseline,
        durationSeconds: Int = 120,
        notes: String = "",
        site: String = "home_lab",
        operatorName: String = ""
    ) {
        self.preset = preset
        self.durationSeconds = durationSeconds
        self.notes = notes
        self.site = site
        self.operatorName = operatorName
    }

    public var layoutTag: String { preset.layoutTag }

    public var deliberateChange: String {
        switch preset {
        case .furnitureMoved:
            return notes.isEmpty ? "one deliberate furniture move" : notes
        default:
            return notes
        }
    }

    /// Session folder name: YYYYMMDDTHHMMSSZ_<A|A2|B|C>_<tag>
    /// Time is required so a second press the same day does not reuse A's bag directory.
    public func sessionID(now: Date = Date(), calendar: Calendar = .current) -> String {
        let f = DateFormatter()
        f.locale = Locale(identifier: "en_US_POSIX")
        f.timeZone = TimeZone(secondsFromGMT: 0)
        f.dateFormat = "yyyyMMdd'T'HHmmss'Z'"
        let stamp = f.string(from: now)
        return "\(stamp)_\(preset.code)_\(preset.layoutTag)"
    }
}

/// One-line log row for the capture console.
public struct CaptureLogLine: Sendable, Equatable, Identifiable {
    public var id: UUID
    public var at: Date
    public var text: String

    public init(id: UUID = UUID(), at: Date = Date(), text: String) {
        self.id = id
        self.at = at
        self.text = text
    }
}

/// House-session (or later bags) that already landed on this Mac.
public struct LandedSession: Sendable, Equatable {
    public var path: String
    public var points: Int?
    public var fields: String?
    public var bytes: Int?
    public var ptp: Bool?
    public var summary: String

    public init(
        path: String,
        points: Int? = nil,
        fields: String? = nil,
        bytes: Int? = nil,
        ptp: Bool? = nil,
        summary: String
    ) {
        self.path = path
        self.points = points
        self.fields = fields
        self.bytes = bytes
        self.ptp = ptp
        self.summary = summary
    }
}
