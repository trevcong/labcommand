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
    private var titleWatch: AnyCancellable?
    private var visibilityObservers: [NSObjectProtocol] = []
    private var keyMonitor: Any?

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
        win.title = appState.dest.windowTitle
        win.contentView = hosting
        win.setContentSize(NSSize(width: 1180, height: 800))
        win.center()
        win.isReleasedWhenClosed = false
        win.minSize = NSSize(width: 1040, height: 700)
        win.appearance = NSAppearance(named: .darkAqua)
        win.backgroundColor = NSColor(red: 0.043, green: 0.051, blue: 0.063, alpha: 1)
        win.makeKeyAndOrderFront(nil)
        window = win
        titleWatch = appState.$dest.receive(on: DispatchQueue.main).sink { [weak self] dest in
            self?.window?.title = dest.windowTitle
        }
        watchVisibility(of: win)

        appState.start()
        installOverlayKeys()
        installStatusItem()
        statusWatch = appState.objectWillChange.receive(on: DispatchQueue.main).sink { [weak self] _ in
            self?.rebuildStatusMenu()
        }
        NSApp.activate(ignoringOtherApps: true)
    }

    /// Live streams only while someone can see it: minimizing, hiding the app, or
    /// covering the window stops it. Closing the window quits the app, which
    /// closes everything anyway.
    private func watchVisibility(of win: NSWindow) {
        let center = NotificationCenter.default
        let triggers: [(Notification.Name, AnyObject)] = [
            (NSWindow.didChangeOcclusionStateNotification, win),
            (NSWindow.didMiniaturizeNotification, win),
            (NSApplication.didHideNotification, NSApp),
        ]
        visibilityObservers = triggers.map { name, object in
            center.addObserver(forName: name, object: object, queue: .main) { [weak self] _ in
                MainActor.assumeIsolated { self?.checkVisibility() }
            }
        }
    }

    private func checkVisibility() {
        guard let window else { return }
        let visible = window.isVisible
            && !window.isMiniaturized
            && !NSApp.isHidden
            && window.occlusionState.contains(.visible)
        if !visible {
            appState.live.viewHidden()
        }
    }

    func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool { true }

    func applicationWillTerminate(_ notification: Notification) {
        if let keyMonitor {
            NSEvent.removeMonitor(keyMonitor)
            self.keyMonitor = nil
        }
        appState.stop()
    }

    /// Left and right step the playhead. The event is consumed so a focused
    /// slider does not step twice. Command, option, and control keep their keys.
    private func installOverlayKeys() {
        keyMonitor = NSEvent.addLocalMonitorForEvents(matching: .keyDown) { [weak self] event in
            let keyCode = event.keyCode
            let flags = event.modifierFlags.intersection(.deviceIndependentFlagsMask).rawValue
            let windowNumber = event.window?.windowNumber
            let textView = event.window?.firstResponder is NSTextView
            let consume = MainActor.assumeIsolated { () -> Bool in
                self?.consumeOverlayKey(
                    keyCode: keyCode,
                    flags: flags,
                    windowNumber: windowNumber,
                    textView: textView
                ) ?? false
            }
            return consume ? nil : event
        }
    }

    private func consumeOverlayKey(keyCode: UInt16, flags: UInt, windowNumber: Int?, textView: Bool) -> Bool {
        guard let window, window.isKeyWindow, window.windowNumber == windowNumber else {
            return false
        }
        let mask = NSEvent.ModifierFlags(rawValue: flags)
        if mask.contains(.command) || mask.contains(.option) || mask.contains(.control) {
            return false
        }
        if textView { return false }
        // Trail replay, in any launch: left/right step one second, space plays or pauses.
        // Only on the Trail tab with a trajectory open; other tabs keep their keys.
        if appState.dest == .trail {
            let trail = appState.trail
            guard trail.trajectory != nil else { return false }
            switch keyCode {
            case 123: trail.step(-1)
            case 124: trail.step(1)
            case 49: trail.toggle()
            default: return false
            }
            return true
        }
        // The snapshot scrubber steps only in an isolated launch.
        guard appState.isolatesNetwork else { return false }
        switch keyCode {
        case 123:
            appState.stepOverlay(-1)
            return true
        case 124:
            appState.stepOverlay(1)
            return true
        default:
            return false
        }
    }

    func validateMenuItem(_ menuItem: NSMenuItem) -> Bool {
        if menuItem.action == #selector(refreshStatus) {
            if appState.isolatesNetwork { return false }
            return appState.dest == .workspace || appState.focusMode
        }
        if menuItem.action == #selector(menuStartPath) {
            return !appState.isolatesNetwork && RigGate.canStartRig(appState.rig).0 && !appState.isActing
        }
        if menuItem.action == #selector(menuRecord) {
            return !appState.isolatesNetwork && RigGate.canRecord(appState.rig).0 && !appState.isActing
        }
        if menuItem.action == #selector(menuStopAll) {
            return !appState.isolatesNetwork && RigGate.canStopAll(appState.rig).0 && !appState.isActing
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
            withTitle: "About LabCommand",
            action: #selector(NSApplication.orderFrontStandardAboutPanel(_:)),
            keyEquivalent: ""
        )
        appMenu.addItem(NSMenuItem.separator())
        appMenu.addItem(
            withTitle: "Quit LabCommand",
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

        // ⌘1 is the first tab in navCases. The buttons do not register these keys too.
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
        // A menu-bar extra stays narrow. The tooltip and the menu carry the full name.
        let item = NSStatusBar.system.statusItem(withLength: NSStatusItem.variableLength)
        let button = item.button
        button?.title = "LAB"
        button?.font = NSFont.monospacedSystemFont(ofSize: 11, weight: .medium)
        button?.toolTip = "LabCommand"
        button?.setAccessibilityLabel("LabCommand")
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
        menu.addItem(header("LabCommand"))
        menu.addItem(.separator())
        let rig = appState.rig
        let dots = appState.statusDots
        menu.addItem(header("Mid-360  \(dots.mid360.state.displayLabel)"))
        menu.addItem(header("JT128    \(dots.jt128.state.displayLabel)"))
        menu.addItem(header("Clocks   \(rig.clock.state.displayLabel)"))
        menu.addItem(header("Record   \(rig.recording.active ? "Live" : "Idle")"))
        if appState.isolatesNetwork {
            menu.addItem(header("Health   Local"))
            menu.addItem(header("Spark    local"))
        } else {
            menu.addItem(header("Health   \(appState.health.label)"))
            if !dots.sparkReachable {
                menu.addItem(header("Spark    offline"))
            }
        }
        if let hz = dots.mid360.pointRateHz, dots.mid360.state == .streaming {
            menu.addItem(header("Mid rate \(TelemetryFormat.hz(hz))"))
        }
        if let hz = dots.jt128.pointRateHz, dots.jt128.state == .streaming {
            menu.addItem(header("JT rate  \(TelemetryFormat.hz(hz))"))
        }
        menu.addItem(.separator())
        let local = appState.isolatesNetwork
        addStatusAction(menu, title: rig.stackRunning ? "Restart Rig" : "Start Rig", action: #selector(menuStartPath), enabled: !local && RigGate.canStartRig(rig).0)
        addStatusAction(menu, title: "Start Recording", action: #selector(menuRecord), enabled: !local && RigGate.canRecord(rig).0)
        addStatusAction(menu, title: "Stop", action: #selector(menuStopAll), enabled: !local && RigGate.canStopAll(rig).0)
        addStatusAction(menu, title: "Diagnose", action: #selector(menuDiagnose), enabled: true)
        addStatusAction(menu, title: appState.focusMode ? "Exit Focus" : "Focus Mode", action: #selector(menuFocus), enabled: true)
        menu.addItem(.separator())
        addStatusAction(menu, title: "Show LabCommand", action: #selector(showWindow), enabled: true)
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
        guard appState.dest == .workspace || appState.focusMode else { return }
        appState.refresh(manual: true)
    }

    @objc private func goDest(_ sender: NSMenuItem) {
        let all = Dest.navCases
        guard sender.tag >= 0, sender.tag < all.count else { return }
        appState.dest = all[sender.tag]
    }
}
