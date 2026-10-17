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

private func repoFile(_ relative: String) -> String {
    let url = URL(fileURLWithPath: FileManager.default.currentDirectoryPath).appendingPathComponent(relative)
    return (try? String(contentsOf: url, encoding: .utf8)) ?? ""
}

/// The window opens on Room, titles itself LabCommand - <tab>, and the event
/// column does not print the wire vocabulary.
func testRoomFirstShell() {
    let app = repoFile("Sources/LabCommand/LabCommandApp.swift")
    let desk = repoFile("Sources/LabCommandCore/RigDesk.swift")
    let state = repoFile("Sources/LabCommand/AppState.swift")
    let chrome = repoFile("Sources/LabCommand/CommandCenterView.swift")
    let room = repoFile("Sources/LabCommandCore/LiveRoomView.swift")
    expect(!app.isEmpty && !desk.isEmpty && !state.isEmpty && !chrome.isEmpty && !room.isEmpty,
           "room-first sources are readable from the repo root")

    expect(!app.contains("Rig command"), "the app does not title the window Rig command")
    expect(app.contains("windowTitle"), "the app takes the window title from Dest")
    expect(app.contains("win.title = appState.dest.windowTitle"), "the initial title is the current destination")
    expect(app.contains("$dest"), "the title follows destination changes")
    expect(!desk.contains("Rig command"), "the rig desk title is not Rig command")
    expect(desk.contains("static let title = \"Rig\""), "the rig desk title is Rig")

    expect(state.contains("var windowTitle: String { \"LabCommand - \\(label)\" }"),
           "window titles are declared on Dest")
    expect(state.contains("static var navCases: [Dest] { [.liveRoom, .settings, .workspace] }"),
           "nav order is Room, Settings, Rig")
    expect(state.contains("case .liveRoom: return \"1\""), "Room is ⌘1")
    expect(state.contains("case .settings: return \"2\""), "Settings stays ⌘2")
    expect(state.contains("case .workspace: return \"3\""), "Rig is ⌘3")
    expect(state.contains("var dest: Dest = .liveRoom"), "a normal launch opens on Room")
    expect(state.contains("launch.isolates && !roomLaunch.opensRoom"), "snapshot isolation is the Rig exception")
    expect(state.contains("dest = .workspace"), "the snapshot tool still has a Rig assignment")

    expect(chrome.contains("case .workspace") && chrome.contains("case .settings") && chrome.contains("case .liveRoom"),
           "every destination still renders")
    expect(!chrome.contains("dual-lidar rig"), "the top bar no longer says dual-lidar rig")
    expect(chrome.contains("room memory"), "the top bar says room memory")

    let banned = [
        "\"candidate\"",
        "\"candidates\"",
        "uncalibrated",
        "no request path",
        "may be outdated",
        "\"evidence ",
        "\"α ",
        " voxels\"",
        "history complete through",
        "cannot request resync",
    ]
    for phrase in banned {
        expect(!room.contains(phrase), "room screen still prints \(phrase)")
    }
    expect(room.contains("case .candidate:"), "the wire state stays .candidate")
}

/// The Dock, menus, and window wordmark say LabCommand. The binary and the
/// ~/Applications install path stay LabCommand.app.
func testVisibleAppName() {
    let plist = repoFile("Info.plist")
    let app = repoFile("Sources/LabCommand/LabCommandApp.swift")
    let chrome = repoFile("Sources/LabCommand/CommandCenterView.swift")
    let build = repoFile("build.sh")
    expect(!plist.isEmpty && !app.isEmpty && !chrome.isEmpty && !build.isEmpty,
           "name sources are readable from the repo root")

    expect(plist.contains("<key>CFBundleDisplayName</key>\n\t<string>LabCommand</string>"),
           "the Dock and Finder name is LabCommand")
    expect(plist.contains("<key>CFBundleName</key>\n\t<string>LabCommand</string>"),
           "the app menu title is LabCommand")
    expect(plist.contains("<key>CFBundleExecutable</key>\n\t<string>LabCommand</string>"),
           "the binary stays LabCommand")
    expect(!plist.contains("<string>Lab</string>"), "the bundle does not name itself Lab")
    expect(plist.contains("LabCommand reads Desktop/lidar-sessions"),
           "the Desktop prompt names LabCommand")

    expect(app.contains("\"About LabCommand\""), "About names LabCommand")
    expect(app.contains("\"Quit LabCommand\""), "Quit names LabCommand")
    expect(app.contains("\"Show LabCommand\""), "the status menu names LabCommand")
    expect(app.contains("button?.title = \"LabCommand\""), "the status item names LabCommand")
    expect(!app.contains("\"About Lab\"") && !app.contains("\"Quit Lab\"") && !app.contains("\"Show Lab\""),
           "menus do not say Lab")

    expect(chrome.contains("Text(\"LabCommand\")"), "the window wordmark is LabCommand")
    expect(chrome.contains(".accessibilityLabel(\"LabCommand\")"), "the wordmark label is LabCommand")
    expect(!chrome.contains("Text(\"Lab\")"), "the window does not say Lab")

    expect(build.contains("APP_NAME=\"LabCommand\""), "the .app filename and binary stay LabCommand")
    expect(build.contains("INSTALL_DIR=\"${HOME}/Applications\""), "install stays ~/Applications")
}
