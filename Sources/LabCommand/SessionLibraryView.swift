import SwiftUI
import AppKit
import LabCommandCore

/// Research notebook over real `~/Desktop/lidar-sessions`.
struct SessionLibraryView: View {
    @EnvironmentObject private var state: AppState
    @State private var noteText = ""

    var body: some View {
        VStack(alignment: .leading, spacing: LabTheme.sectionGap) {
            filterBar
            if let cmp = state.compareResult {
                compareTable(cmp)
            }
            sessionTable
            if let stem = state.noteTargetStem,
               let rec = state.library.first(where: { $0.stem == stem }) {
                noteEditor(rec)
            }
        }
        .accessibilityIdentifier("sessionLibrary")
    }

    private var filterBar: some View {
        HStack(spacing: 8) {
            TextField("Search files, notes, tags", text: $state.libraryQuery)
                .textFieldStyle(.plain)
                .font(LabTheme.t(13))
                .foregroundStyle(LabTheme.text)
                .padding(.horizontal, 10)
                .padding(.vertical, 6)
                .background(
                    RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                        .fill(LabTheme.well)
                )
                .overlay(
                    RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                        .stroke(LabTheme.line, lineWidth: 1)
                )
            ForEach(["A", "A2", "B", "C"], id: \.self) { tag in
                let on = state.libraryTag == tag
                Button(tag) {
                    state.libraryTag = on ? nil : tag
                }
                .buttonStyle(GhostButton())
                .opacity(on ? 1 : 0.7)
            }
            Text("\(state.filteredLibrary.count)")
                .font(LabTheme.mono(11))
                .foregroundStyle(LabTheme.faint)
        }
    }

    private var sessionTable: some View {
        VStack(alignment: .leading, spacing: 0) {
            HStack(spacing: 10) {
                Text("File").frame(minWidth: 180, alignment: .leading)
                Text("Points").frame(width: 80, alignment: .trailing)
                Text("Size").frame(width: 72, alignment: .trailing)
                Text("PTP").frame(width: 64, alignment: .leading)
                Text("Quality").frame(minWidth: 120, alignment: .leading)
                Text("").frame(maxWidth: .infinity)
            }
            .font(LabTheme.label())
            .foregroundStyle(LabTheme.faint)
            .textCase(.uppercase)
            .tracking(0.6)
            .padding(.vertical, 8)

            if state.filteredLibrary.isEmpty {
                Text("No sessions match.")
                    .font(LabTheme.t(12))
                    .foregroundStyle(LabTheme.faint)
                    .padding(.vertical, 12)
            } else {
                ForEach(state.filteredLibrary) { rec in
                    sessionRow(rec)
                    Rectangle().fill(LabTheme.line).frame(height: 1)
                }
            }
        }
    }

    private func sessionRow(_ rec: SessionRecord) -> some View {
        let selected = state.compareLeft == rec.path || state.compareRight == rec.path
        return VStack(alignment: .leading, spacing: 8) {
            HStack(spacing: 10) {
                Text(rec.filename)
                    .font(LabTheme.mono(12))
                    .foregroundStyle(LabTheme.text)
                    .frame(minWidth: 180, alignment: .leading)
                    .lineLimit(1)
                Text(rec.pointsDisplay)
                    .font(LabTheme.mono(12))
                    .foregroundStyle(rec.points == nil ? LabTheme.faint : LabTheme.dim)
                    .frame(width: 80, alignment: .trailing)
                Text(rec.bytesDisplay)
                    .font(LabTheme.mono(12))
                    .foregroundStyle(rec.bytes == nil ? LabTheme.faint : LabTheme.dim)
                    .frame(width: 72, alignment: .trailing)
                Text(rec.ptpDisplay)
                    .font(LabTheme.mono(11))
                    .foregroundStyle(rec.ptp == nil ? LabTheme.faint : LabTheme.dim)
                    .frame(width: 64, alignment: .leading)
                Text(rec.quality.summary)
                    .font(LabTheme.t(12))
                    .foregroundStyle(LabTheme.dim)
                    .frame(minWidth: 120, alignment: .leading)
                Spacer(minLength: 8)
            }
            HStack(spacing: 6) {
                Button("Reveal") { reveal(rec.path) }
                    .buttonStyle(GhostButton())
                Button("Open in Foxglove") { openFoxglove() }
                    .buttonStyle(GhostButton())
                    .disabled(!state.rig.viewTunnelUp)
                    .opacity(state.rig.viewTunnelUp ? 1 : 0.38)
                    .help(state.rig.viewTunnelUp
                          ? CaptureRecipe.viewConnectURL()
                          : "Tunnel off — Start path first")
                Button("Note") {
                    state.noteTargetStem = rec.stem
                    noteText = rec.notes
                }
                .buttonStyle(GhostButton())
                Button(selected ? "Compared" : "Compare") {
                    state.toggleCompare(rec.path)
                }
                .buttonStyle(GhostButton())
                if !rec.tags.isEmpty {
                    Text(rec.tags.joined(separator: " "))
                        .font(LabTheme.mono(11))
                        .foregroundStyle(LabTheme.accent)
                }
                if !rec.notes.isEmpty {
                    Text(rec.notes)
                        .font(LabTheme.t(11))
                        .foregroundStyle(LabTheme.faint)
                        .lineLimit(1)
                }
            }
        }
        .padding(.vertical, 10)
    }

    private func noteEditor(_ rec: SessionRecord) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            SectionLabel(text: "Operator note · \(rec.stem)")
            TextField("What moved, weather, doors", text: $noteText, axis: .vertical)
                .lineLimit(3...6)
                .textFieldStyle(.plain)
                .font(LabTheme.t(13))
                .foregroundStyle(LabTheme.text)
                .padding(10)
                .background(
                    RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                        .fill(LabTheme.well)
                )
                .overlay(
                    RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                        .stroke(LabTheme.line, lineWidth: 1)
                )
            HStack {
                Button("Save note") { state.attachNote(to: rec, text: noteText) }
                    .buttonStyle(QuietButton())
                Button("Cancel") { state.noteTargetStem = nil }
                    .buttonStyle(GhostButton())
            }
        }
    }

    private func compareTable(_ cmp: SessionCompare) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            SectionLabel(text: "Compare")
            HStack {
                Text(cmp.left.filename)
                    .font(LabTheme.mono(11))
                    .foregroundStyle(LabTheme.dim)
                    .frame(maxWidth: .infinity, alignment: .leading)
                Text(cmp.right.filename)
                    .font(LabTheme.mono(11))
                    .foregroundStyle(LabTheme.dim)
                    .frame(maxWidth: .infinity, alignment: .leading)
            }
            ForEach(cmp.rows) { row in
                HStack(alignment: .firstTextBaseline) {
                    Text(row.label)
                        .font(LabTheme.mono(11))
                        .foregroundStyle(LabTheme.faint)
                        .frame(width: 72, alignment: .leading)
                    Text(row.left)
                        .font(LabTheme.mono(12))
                        .foregroundStyle(LabTheme.text)
                        .frame(maxWidth: .infinity, alignment: .leading)
                    Text(row.right)
                        .font(LabTheme.mono(12))
                        .foregroundStyle(LabTheme.text)
                        .frame(maxWidth: .infinity, alignment: .leading)
                }
                .padding(.vertical, 3)
            }
        }
        .padding(12)
        .background(
            RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                .fill(LabTheme.surface)
        )
        .overlay(
            RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                .stroke(LabTheme.line, lineWidth: 1)
        )
    }

    private func reveal(_ path: String) {
        NSWorkspace.shared.activateFileViewerSelecting([URL(fileURLWithPath: path)])
    }

    private func openFoxglove() {
        guard state.rig.viewTunnelUp,
              let url = URL(string: CaptureRecipe.viewConnectURL()) else { return }
        NSWorkspace.shared.open(url)
    }
}
