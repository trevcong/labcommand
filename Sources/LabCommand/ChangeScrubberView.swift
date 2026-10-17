import SwiftUI
import LabCommandCore

/// Playhead over one occupancy diff. Copy comes from `ChangeOverlay`.
struct ChangeScrubberView: View {
    @EnvironmentObject private var state: AppState

    var body: some View {
        let slider = state.scrubberSlider
        VStack(alignment: .leading, spacing: 6) {
            HStack(spacing: 8) {
                Text(state.scrubberBefore)
                    .font(LabTheme.mono(11))
                    .foregroundStyle(LabTheme.dim)
                    .frame(width: 136, alignment: .leading)
                Slider(
                    value: Binding(
                        get: { Double(state.scrubberSlider.value) },
                        set: { state.setOverlayCount(Int($0.rounded())) }
                    ),
                    in: Double(slider.lower)...Double(slider.upper),
                    step: 1
                )
                .disabled(!slider.enabled)
                .opacity(slider.enabled ? 1 : LabTheme.disabledOpacity)
                .accessibilityLabel("Change timeline")
                Text(state.scrubberAfter)
                    .font(LabTheme.mono(11))
                    .foregroundStyle(LabTheme.dim)
                    .frame(width: 136, alignment: .trailing)
            }
            if !state.scrubberChips.isEmpty {
                HStack(spacing: 6) {
                    ForEach(state.scrubberChips) { chip in
                        chipButton(chip)
                    }
                }
            }
            Text(state.scrubberCaption)
                .font(LabTheme.mono(11))
                .foregroundStyle(LabTheme.dim)
                .lineLimit(2)
                .frame(maxWidth: .infinity, alignment: .leading)
                .accessibilityLabel("Change caption")
        }
    }

    private func chipButton(_ chip: ChangeOverlay.Chip) -> some View {
        let on = state.overlayShows(chip.kind)
        let tint = Color(red: Double(chip.red), green: Double(chip.green), blue: Double(chip.blue))
        return Button {
            state.setOverlayKind(chip.kind, shown: !on)
        } label: {
            Text(chip.title)
                .font(LabTheme.mono(11))
                .foregroundStyle(on ? tint : LabTheme.dim)
                .padding(.horizontal, 8)
                .padding(.vertical, 3)
                .background(Capsule().fill(LabTheme.well))
                .overlay(
                    Capsule().stroke(on ? tint : LabTheme.line, lineWidth: 1)
                )
        }
        .buttonStyle(.plain)
        .opacity(on ? 1 : LabTheme.disabledOpacity)
        .accessibilityLabel(chip.title)
    }
}
