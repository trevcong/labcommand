import SwiftUI

struct QuietButton: ButtonStyle {
    func makeBody(configuration: Configuration) -> some View {
        configuration.label
            .padding(.horizontal, 10)
            .padding(.vertical, 5)
            .foregroundStyle(LabTheme.dim)
            .background(
                RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                    .stroke(LabTheme.lineStrong, lineWidth: 1)
                    .background(
                        RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                            .fill(configuration.isPressed ? LabTheme.surfaceRaised : Color.clear)
                    )
            )
    }
}

struct PrimaryButton: ButtonStyle {
    var emphasized: Bool = false
    func makeBody(configuration: Configuration) -> some View {
        configuration.label
            .font(LabTheme.t(13, .semibold))
            .foregroundStyle(emphasized ? LabTheme.text : LabTheme.dim)
            .padding(.horizontal, 16)
            .padding(.vertical, 9)
            .background(
                RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                    .fill(emphasized ? LabTheme.accentMuted : LabTheme.surfaceRaised)
            )
            .overlay(
                RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                    .stroke(emphasized ? LabTheme.accent.opacity(0.55) : LabTheme.lineStrong, lineWidth: 1)
            )
            .opacity(configuration.isPressed ? 0.75 : 1)
    }
}

struct StopButton: ButtonStyle {
    func makeBody(configuration: Configuration) -> some View {
        configuration.label
            .font(LabTheme.t(13, .semibold))
            .foregroundStyle(LabTheme.bad)
            .padding(.horizontal, 16)
            .padding(.vertical, 9)
            .background(
                RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                    .fill(LabTheme.bad.opacity(0.10))
            )
            .overlay(
                RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                    .stroke(LabTheme.bad.opacity(0.45), lineWidth: 1)
            )
            .opacity(configuration.isPressed ? 0.75 : 1)
    }
}

struct HeroButton: ButtonStyle {
    var kind: Kind = .primary
    enum Kind { case primary, record, stop, muted }

    func makeBody(configuration: Configuration) -> some View {
        let fill: Color = {
            switch kind {
            case .primary: return LabTheme.accent
            case .record: return LabTheme.good
            case .stop: return LabTheme.bad
            case .muted: return LabTheme.surfaceRaised
            }
        }()
        let fg: Color = {
            switch kind {
            case .muted: return LabTheme.dim
            default: return Color.white
            }
        }()
        // Contract §Motion dims a disabled control to 0.38. That belongs on the
        // LABEL: dimming the whole control erased the shape, so the operator
        // could not see where the button was, only that something was missing.
        return configuration.label
            .font(LabTheme.t(15, .semibold))
            .foregroundStyle(fg.opacity(kind == .muted ? LabTheme.disabledOpacity : 1))
            .padding(.horizontal, 28)
            .padding(.vertical, 13)
            .background(
                RoundedRectangle(cornerRadius: LabTheme.heroRadius, style: .continuous)
                    .fill(fill)
            )
            .overlay(
                RoundedRectangle(cornerRadius: LabTheme.heroRadius, style: .continuous)
                    .stroke(kind == .muted ? LabTheme.lineStrong : Color.clear, lineWidth: 1)
            )
            .opacity(configuration.isPressed ? 0.82 : 1)
    }
}

struct GhostButton: ButtonStyle {
    func makeBody(configuration: Configuration) -> some View {
        configuration.label
            .font(LabTheme.t(11, .medium))
            .foregroundStyle(LabTheme.dim)
            .padding(.horizontal, 8)
            .padding(.vertical, 4)
            .background(
                RoundedRectangle(cornerRadius: 4, style: .continuous)
                    .fill(configuration.isPressed ? LabTheme.surfaceRaised : Color.clear)
            )
            .overlay(
                RoundedRectangle(cornerRadius: 4, style: .continuous)
                    .stroke(LabTheme.line, lineWidth: 1)
            )
    }
}
