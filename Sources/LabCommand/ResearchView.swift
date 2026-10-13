import SwiftUI

/// Recorded takes on this Mac. No empty research berths.
struct ResearchView: View {
    @EnvironmentObject private var state: AppState

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: LabTheme.sectionGap) {
                Text("Takes")
                    .font(LabTheme.t(LabTheme.titleSize, .semibold))
                    .foregroundStyle(LabTheme.text)
                Text("Bags and sidecars already on this Mac (~/Desktop/lidar-sessions). Record new ones on the Record tab. Compare and notes live here.")
                    .font(LabTheme.t(13))
                    .foregroundStyle(LabTheme.dim)
                    .fixedSize(horizontal: false, vertical: true)
                if let track = state.currentTrack {
                    TrajectoryCard(track: track)
                        .id("takes-" + track.session)
                }
                SessionLibraryView()
            }
            .padding(LabTheme.pad)
            .frame(maxWidth: 880, alignment: .leading)
            .frame(maxWidth: .infinity, alignment: .center)
        }
        .background(LabTheme.bg)
    }
}
