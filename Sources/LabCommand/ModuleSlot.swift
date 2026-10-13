import SwiftUI

/// Shared empty/bound content berth. Unbound = title + em dash. No marketing.
struct ModuleSlot: View {
    let title: String
    var isBound: Bool = false
    var showPlus: Bool = false
    var minHeight: CGFloat = LabTheme.slotMinH
    /// Compact one-line fact when bound (preferred over freeform content).
    var status: String? = nil
    var content: AnyView? = nil

    var body: some View {
        VStack(alignment: .leading, spacing: 8) {
            Text(title)
                .font(LabTheme.t(12, .medium))
                .foregroundStyle(LabTheme.dim)
                .lineLimit(1)

            if isBound, let status, !status.isEmpty {
                Text(status)
                    .font(LabTheme.mono(12))
                    .foregroundStyle(LabTheme.text)
                    .lineLimit(3)
                    .fixedSize(horizontal: false, vertical: true)
                    .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .topLeading)
            } else if isBound, let content {
                content
                    .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .topLeading)
            } else if showPlus {
                Image(systemName: "plus")
                    .font(.system(size: 11, weight: .medium))
                    .foregroundStyle(LabTheme.faint)
                    .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .center)
            } else {
                Text("—")
                    .font(LabTheme.t(13))
                    .foregroundStyle(LabTheme.faint)
                    .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .topLeading)
            }
        }
        .padding(12)
        .frame(maxWidth: .infinity, minHeight: minHeight, alignment: .topLeading)
        .background(
            RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                .fill(LabTheme.surface.opacity(0.55))
        )
        .overlay(
            RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                .stroke(LabTheme.line, lineWidth: 1)
        )
        .accessibilityElement(children: .combine)
        .accessibilityLabel(isBound ? "\(title), \(status ?? "bound")" : "\(title), empty")
    }
}

/// Full-width list berth for Research-style rows.
struct ModuleBerth: View {
    let title: String
    var isBound: Bool = false
    /// One-line status from real files; shown after "bound".
    var status: String? = nil

    var body: some View {
        HStack(alignment: .firstTextBaseline, spacing: 12) {
            Text(title)
                .font(LabTheme.t(13, .medium))
                .foregroundStyle(LabTheme.text)
            Spacer(minLength: 8)
            if isBound {
                VStack(alignment: .trailing, spacing: 2) {
                    Text("bound")
                        .font(LabTheme.t(12, .medium))
                        .foregroundStyle(LabTheme.dim)
                    if let status, !status.isEmpty {
                        Text(status)
                            .font(LabTheme.mono(11))
                            .foregroundStyle(LabTheme.faint)
                            .lineLimit(1)
                    }
                }
            } else {
                Text("—")
                    .font(LabTheme.t(13))
                    .foregroundStyle(LabTheme.faint)
            }
        }
        .padding(.horizontal, 14)
        .frame(maxWidth: .infinity, minHeight: LabTheme.berthH, alignment: .leading)
        .background(LabTheme.surface.opacity(0.45))
        .overlay(
            Rectangle()
                .stroke(LabTheme.line, lineWidth: 1)
        )
        .accessibilityLabel(isBound ? "\(title), bound, \(status ?? "")" : "\(title), empty")
    }
}

/// Section label above a group of slots.
struct SectionLabel: View {
    let text: String

    var body: some View {
        Text(text)
            .font(LabTheme.t(11, .medium))
            .foregroundStyle(LabTheme.faint)
            .textCase(.uppercase)
            .tracking(0.6)
    }
}
