import SwiftUI
import LabCommandCore

/// v0.4 instrument console. Hex contract: design-refs/DESIGN-VISION.md
enum LabTheme {
    // MARK: Surfaces
    static let bg = Color(red: 0.043, green: 0.051, blue: 0.063)            // #0B0D10
    static let sidebar = Color(red: 0.031, green: 0.039, blue: 0.047)       // #080A0C
    static let surface = Color(red: 0.071, green: 0.086, blue: 0.102)       // #12161A
    static let surfaceRaised = Color(red: 0.094, green: 0.118, blue: 0.141) // #181E24
    static let well = Color(red: 0.055, green: 0.071, blue: 0.086)          // #0E1216

    // MARK: Borders
    static let line = Color(red: 0.141, green: 0.169, blue: 0.196)          // #242B32
    static let lineStrong = Color(red: 0.204, green: 0.235, blue: 0.271)    // #343C45

    // MARK: Text
    static let text = Color(red: 0.902, green: 0.918, blue: 0.937)          // #E6EAEF
    static let dim = Color(red: 0.545, green: 0.584, blue: 0.631)           // #8B95A1
    static let faint = Color(red: 0.361, green: 0.400, blue: 0.439)         // #5C6670

    // MARK: Accent (cool steel — rare)
    static let accent = Color(red: 0.290, green: 0.561, blue: 0.722)        // #4A8FB8
    static let accentMuted = Color(red: 0.290, green: 0.561, blue: 0.722).opacity(0.14)

    // MARK: Status (pips + 1px strokes)
    static let good = Color(red: 0.247, green: 0.639, blue: 0.478)          // #3FA37A
    static let warn = Color(red: 0.769, green: 0.627, blue: 0.290)          // #C4A04A
    static let bad = Color(red: 0.769, green: 0.353, blue: 0.353)           // #C45A5A
    static let idle = Color(red: 0.290, green: 0.329, blue: 0.369)          // #4A545E
    static let unknown = Color(red: 0.227, green: 0.259, blue: 0.290)       // #3A424A

    // MARK: Layout — DESIGN-VISION.md §"Spacing and shape" is the contract.
    // These had drifted to webpage proportions (56/28/22/10/8), which is what
    // made a dense instrument read like a settings page.
    static let sidebarW: CGFloat = 196
    static let topBarH: CGFloat = 40
    static let pad: CGFloat = 20
    static let sectionGap: CGFloat = 20
    static let railGap: CGFloat = 8
    static let slotMinH: CGFloat = 96
    static let berthH: CGFloat = 56
    static let radius: CGFloat = 7
    static let controlRadius: CGFloat = 5
    static let heroRadius: CGFloat = 7
    static let pip: CGFloat = 6
    static let motion: Double = 0.18
    /// Chrome cap. Destination titles must not exceed this (contract §Typography).
    static let titleSize: CGFloat = 15
    /// Contract §Motion: disabled controls sit at 0.38 and never pulse.
    static let disabledOpacity: Double = 0.38

    static func status(_ s: ConnectionStatus) -> Color {
        switch s {
        case .connected: return good
        case .disconnected, .error: return bad
        case .unknown: return idle
        }
    }

    static func stageRig(_ s: RigStageState) -> Color {
        switch s {
        case .running: return good
        case .blocked: return bad
        case .idle: return idle
        case .unknown: return unknown
        }
    }

    static func band(_ b: RigBand) -> Color {
        switch b {
        case .ready: return good
        case .watch: return warn
        // Parked is deliberate and calm — steel accent, never a warning.
        case .parked: return accent
        case .blocked: return bad
        case .unavailable: return faint
        }
    }

    static func t(_ size: CGFloat, _ w: Font.Weight = .regular) -> Font {
        .system(size: size, weight: w, design: .default)
    }

    static func mono(_ size: CGFloat = 11) -> Font {
        .system(size: size, weight: .regular, design: .monospaced)
    }

    static func label(_ size: CGFloat = 10.5) -> Font {
        .system(size: size, weight: .medium, design: .default)
    }
}

struct InstrumentWell<Content: View>: View {
    var minHeight: CGFloat = 0
    var stroke: Color = LabTheme.line
    @ViewBuilder var content: () -> Content

    var body: some View {
        content()
            .padding(12)
            .frame(maxWidth: .infinity, minHeight: minHeight, alignment: .topLeading)
            .background(
                RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                    .fill(LabTheme.surface)
            )
            .overlay(
                RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                    .stroke(stroke, lineWidth: 1)
            )
    }
}
