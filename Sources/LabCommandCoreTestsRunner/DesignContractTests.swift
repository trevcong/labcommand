import Foundation
import LabCommandCore

// design-refs/DESIGN-VISION.md is the visual contract. It drifted once — titles
// grew to 22/26pt against a stated 15pt cap and every spacing token inflated,
// which is what made a dense instrument read like a settings page. These pin the
// numbers so the drift is a test failure, not a slow slide.

private func labSources() -> [(name: String, body: String)] {
    let dir = URL(fileURLWithPath: FileManager.default.currentDirectoryPath)
        .appendingPathComponent("Sources/LabCommand")
    let names = (try? FileManager.default.contentsOfDirectory(atPath: dir.path)) ?? []
    return names.filter { $0.hasSuffix(".swift") }.compactMap { n in
        guard let b = try? String(contentsOf: dir.appendingPathComponent(n), encoding: .utf8) else {
            return nil
        }
        return (n, b)
    }
}

/// Contract §Typography: "Destination title | 15 | semibold | Chrome cap — do
/// not exceed" and "Never display a live Hz / byte figure larger than 13pt."
func testNoTypeScaleExceedsChromeCap() {
    let files = labSources()
    expect(!files.isEmpty, "found LabCommand sources to scan")

    for (name, body) in files {
        for size in 16...48 {
            let literal = "LabTheme.t(\(size)"
            expect(
                !body.contains(literal),
                "\(name) uses \(size)pt — contract caps chrome type at 15pt (LabTheme.titleSize)"
            )
        }
    }
}

/// Contract §"Spacing and shape". These are the exact table values.
func testSpacingTokensMatchContract() {
    let dir = URL(fileURLWithPath: FileManager.default.currentDirectoryPath)
    let theme = dir.appendingPathComponent("Sources/LabCommand/Theme.swift")
    guard let body = try? String(contentsOf: theme, encoding: .utf8) else {
        expect(false, "Theme.swift missing")
        return
    }
    let pinned: [(String, String)] = [
        ("sidebarW", "196"),
        ("topBarH", "40"),
        ("pad", "20"),
        ("sectionGap", "20"),
        ("radius", "7"),
        ("controlRadius", "5"),
        ("pip", "6"),
        ("titleSize", "15"),
    ]
    for (token, value) in pinned {
        expect(
            body.contains("static let \(token): CGFloat = \(value)"),
            "Theme.\(token) must be \(value) per DESIGN-VISION.md §Spacing and shape"
        )
    }
    // §Motion: "Disabled controls: opacity 0.38, no pulse."
    expect(body.contains("disabledOpacity: Double = 0.38"), "disabled controls sit at 0.38")
}

/// Contract §Banned: "Empty inspector columns", "coming soon", KPI/thesis chrome.
/// Product's Offer/Pipeline/Accounts berths were exactly that, and the
/// destination rendered Settings — a nav entry that went nowhere.
func testNoPlaceholderScaffolding() {
    let files = labSources()
    let names = Set(files.map(\.name))
    expect(!names.contains("ProductView.swift"), "ProductView was empty berths — banned chrome")
    expect(!names.contains("WorkspaceView.swift"), "WorkspaceView was a passthrough nothing referenced")

    for (name, body) in files {
        expect(!body.localizedCaseInsensitiveContains("coming soon"), "\(name) ships 'coming soon'")
        expect(!body.contains("not wired yet"), "\(name) admits a control is fake — wire it or drop it")
    }

    // Every nav destination must render its own view, not stand in for another.
    let dir = URL(fileURLWithPath: FileManager.default.currentDirectoryPath)
    let chrome = dir.appendingPathComponent("Sources/LabCommand/CommandCenterView.swift")
    if let body = try? String(contentsOf: chrome, encoding: .utf8) {
        expect(!body.contains("case .product:"), "the product destination is gone from the switch")
    }
    let stateFile = dir.appendingPathComponent("Sources/LabCommand/AppState.swift")
    if let body = try? String(contentsOf: stateFile, encoding: .utf8) {
        expect(!body.contains("case product"), "Dest.product is gone from the enum")
    }
}
