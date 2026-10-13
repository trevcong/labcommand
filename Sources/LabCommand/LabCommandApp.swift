import SwiftUI
import AppKit
import Combine
import LabCommandCore

@main
enum LabCommandMain {
    static func main() {
        let app = NSApplication.shared
        let delegate = AppDelegate()
        app.delegate = delegate
        app.setActivationPolicy(.regular)
        app.run()
    }
}

@MainActor
final class AppDelegate: NSObject, NSApplicationDelegate, NSMenuItemValidation {
    private var window: NSWindow?
    private let appState = AppState()
    private var statusItem: NSStatusItem?
    private var statusWatch: AnyCancellable?

    func applicationDidFinishLaunching(_ notification: Notification) {
        buildMainMenu()

        let root = CommandCenterView()
            .environmentObject(appState)

        let hosting = NSHostingView(rootView: root)
        let win = NSWindow(
            contentRect: NSRect(x: 0, y: 0, width: 1040, height: 680),
            styleMask: [.titled, .closable, .miniaturizable, .resizable],
            backing: .buffered,
            defer: false
        )
        win.title = "Rig command"
        win.contentView = hosting
        win.setContentSize(NSSize(width: 1180, height: 800))
        win.center()
        win.isReleasedWhenClosed = false
        win.minSize = NSSize(width: 1040, height: 700)
        win.appearance = NSAppearance(named: .darkAqua)
        win.backgroundColor = NSColor(red: 0.043, green: 0.051, blue: 0.063, alpha: 1)
        win.makeKeyAndOrderFront(nil)
        window = win

        appState.start()
        installStatusItem()
        statusWatch = appState.objectWillChange.receive(on: DispatchQueue.main).sink { [weak self] _ in
            self?.rebuildStatusMenu()
        }
        NSApp.activate(ignoringOtherApps: true)
    }

    func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool { true }

    func applicationWillTerminate(_ notification: Notification) {
        appState.stop()
    }

    func validateMenuItem(_ menuItem: NSMenuItem) -> Bool {
        if menuItem.action == #selector(refreshStatus) {
            return appState.dest == .devices || appState.dest == .workspace || appState.focusMode
        }
        if menuItem.action == #selector(menuStartPath) {
            return RigGate.canStartRig(appState.rig).0 && !appState.isActing
        }
        if menuItem.action == #selector(menuRecord) {
            return RigGate.canRecord(appState.rig).0 && !appState.isActing
        }
        if menuItem.action == #selector(menuStopAll) {
            return RigGate.canStopAll(appState.rig).0 && !appState.isActing
        }
        if menuItem.action == #selector(menuDiagnose) {
            return true
        }
        if menuItem.action == #selector(menuFocus) {
            return true
        }
        return true
    }

    private func buildMainMenu() {
        let mainMenu = NSMenu()

        let appMenuItem = NSMenuItem()
        mainMenu.addItem(appMenuItem)
        let appMenu = NSMenu()
        appMenuItem.submenu = appMenu
        appMenu.addItem(
            withTitle: "About Lab",
            action: #selector(NSApplication.orderFrontStandardAboutPanel(_:)),
            keyEquivalent: ""
        )
        appMenu.addItem(NSMenuItem.separator())
        appMenu.addItem(
            withTitle: "Quit Lab",
            action: #selector(NSApplication.terminate(_:)),
            keyEquivalent: "q"
        )

        let editItem = NSMenuItem()
        mainMenu.addItem(editItem)
        let editMenu = NSMenu(title: "Edit")
        editItem.submenu = editMenu
        editMenu.addItem(withTitle: "Copy", action: #selector(NSText.copy(_:)), keyEquivalent: "c")
        editMenu.addItem(withTitle: "Select All", action: #selector(NSText.selectAll(_:)), keyEquivalent: "a")

        let viewItem = NSMenuItem()
        mainMenu.addItem(viewItem)
        let viewMenu = NSMenu(title: "View")
        viewItem.submenu = viewMenu

        for (idx, d) in Dest.navCases.enumerated() {
            let item = NSMenuItem(
                title: d.label,
                action: #selector(goDest(_:)),
                keyEquivalent: d.shortcutKey
            )
            item.target = self
            item.tag = idx
            item.keyEquivalentModifierMask = [.command]
            viewMenu.addItem(item)
        }

        viewMenu.addItem(NSMenuItem.separator())
        let refresh = NSMenuItem(title: "Refresh", action: #selector(refreshStatus), keyEquivalent: "r")
        refresh.target = self
        viewMenu.addItem(refresh)
        let focus = NSMenuItem(title: "Focus Mode", action: #selector(menuFocus), keyEquivalent: "f")
        focus.target = self
        focus.keyEquivalentModifierMask = [.command, .shift]
        viewMenu.addItem(focus)

        let captureItem = NSMenuItem()
        mainMenu.addItem(captureItem)
        let captureMenu = NSMenu(title: "Capture")
        captureItem.submenu = captureMenu
        let startPath = NSMenuItem(title: "Start Rig", action: #selector(menuStartPath), keyEquivalent: "\r")
        startPath.target = self
        startPath.keyEquivalentModifierMask = [.command]
        captureMenu.addItem(startPath)
        let rec = NSMenuItem(title: "Start Recording", action: #selector(menuRecord), keyEquivalent: "r")
        rec.target = self
        rec.keyEquivalentModifierMask = [.command, .shift]
        captureMenu.addItem(rec)
        let stop = NSMenuItem(title: "Stop", action: #selector(menuStopAll), keyEquivalent: ".")
        stop.target = self
        captureMenu.addItem(stop)
        captureMenu.addItem(NSMenuItem.separator())
        let diag = NSMenuItem(title: "Diagnose", action: #selector(menuDiagnose), keyEquivalent: "d")
        diag.target = self
        diag.keyEquivalentModifierMask = [.command, .shift]
        captureMenu.addItem(diag)

        NSApp.mainMenu = mainMenu
    }

    private func installStatusItem() {
        let item = NSStatusBar.system.statusItem(withLength: NSStatusItem.variableLength)
        item.button?.title = "LAB"
        item.button?.font = NSFont.monospacedSystemFont(ofSize: 11, weight: .medium)
        item.menu = makeStatusMenu()
        statusItem = item
        rebuildStatusMenu()
    }

    private func makeStatusMenu() -> NSMenu {
        let menu = NSMenu()
        menu.autoenablesItems = false
        return menu
    }

    private func rebuildStatusMenu() {
        guard let menu = statusItem?.menu else { return }
        menu.removeAllItems()
        let rig = appState.rig
        menu.addItem(header("Mid-360  \(rig.mid360.state.displayLabel)"))
        menu.addItem(header("JT128    \(rig.jt128.state.displayLabel)"))
        menu.addItem(header("Clocks   \(rig.clock.state.displayLabel)"))
        menu.addItem(header("Record   \(rig.recording.active ? "Live" : "Idle")"))
        menu.addItem(header("Health   \(appState.health.label)"))
        if !rig.sparkReachable {
            menu.addItem(header("Spark    offline"))
        }
        if let hz = rig.mid360.pointRateHz, rig.mid360.state == .streaming {
            menu.addItem(header("Mid rate \(TelemetryFormat.hz(hz))"))
        }
        if let hz = rig.jt128.pointRateHz, rig.jt128.state == .streaming {
            menu.addItem(header("JT rate  \(TelemetryFormat.hz(hz))"))
        }
        menu.addItem(.separator())
        addStatusAction(menu, title: rig.stackRunning ? "Restart Rig" : "Start Rig", action: #selector(menuStartPath), enabled: RigGate.canStartRig(rig).0)
        addStatusAction(menu, title: "Start Recording", action: #selector(menuRecord), enabled: RigGate.canRecord(rig).0)
        addStatusAction(menu, title: "Stop", action: #selector(menuStopAll), enabled: RigGate.canStopAll(rig).0)
        addStatusAction(menu, title: "Diagnose", action: #selector(menuDiagnose), enabled: true)
        addStatusAction(menu, title: appState.focusMode ? "Exit Focus" : "Focus Mode", action: #selector(menuFocus), enabled: true)
        menu.addItem(.separator())
        addStatusAction(menu, title: "Show Lab", action: #selector(showWindow), enabled: true)
    }

    private func header(_ title: String) -> NSMenuItem {
        let item = NSMenuItem(title: title, action: nil, keyEquivalent: "")
        item.isEnabled = false
        return item
    }

    private func addStatusAction(_ menu: NSMenu, title: String, action: Selector, enabled: Bool) {
        let item = NSMenuItem(title: title, action: action, keyEquivalent: "")
        item.target = self
        item.isEnabled = enabled && !appState.isActing
        menu.addItem(item)
    }

    @objc private func menuStartPath() { appState.startRig() }
    @objc private func menuRecord() { appState.startRecord() }
    @objc private func menuStopAll() { appState.stopAll() }
    @objc private func menuDiagnose() {
        appState.dest = .workspace
        appState.diagnose()
        showWindow()
    }
    @objc private func menuFocus() { appState.toggleFocus() }
    @objc private func showWindow() {
        window?.makeKeyAndOrderFront(nil)
        NSApp.activate(ignoringOtherApps: true)
    }

    @objc private func refreshStatus() {
        guard appState.dest == .devices || appState.dest == .workspace || appState.focusMode else { return }
        appState.refresh(manual: true)
    }

    @objc private func goDest(_ sender: NSMenuItem) {
        let all = Dest.navCases
        guard sender.tag >= 0, sender.tag < all.count else { return }
        appState.dest = all[sender.tag]
    }
}
