import Darwin
import Foundation

/// Why a typed room-memory address was not saved.
public enum RoomMemoryEndpointError: Error, Equatable, Sendable {
    case empty
    case unsupportedScheme
    case missingHost
    case missingPort
    case badPort
    case userinfo
    case path
    case query
    case fragment
    case ambiguousAddress
    case unparseable

    public var message: String {
        switch self {
        case .empty: return "Enter a WebSocket URL."
        case .unsupportedScheme: return "Use ws:// or wss://."
        case .missingHost: return "Enter a host name or IP address."
        case .missingPort: return "Include a port. A missing port is not port 80."
        case .badPort: return "Port must be from 1 to 65535."
        case .userinfo: return "Drop the username and password from the URL."
        case .path: return "The endpoint has no path."
        case .query: return "The endpoint has no query."
        case .fragment: return "The endpoint has no fragment."
        case .ambiguousAddress: return "Write the IPv4 address as four decimal numbers, without leading zeros or a trailing dot."
        case .unparseable: return "That is not a WebSocket URL."
        }
    }
}

/// What `commit` did. A downgrade from `wss://` to `ws://` is never `.saved` until the caller confirms it.
public enum RoomEndpointCommitResult: Equatable, Sendable {
    case saved
    case rejected(String)
    case needsCleartextConfirmation
}

/// Whether a URL may be dialed. `blocked` carries the line the Room screen shows.
public enum RoomCleartextDecision: Equatable, Sendable {
    case allowed
    case blocked(String)
}

/// What to do with one HTTP redirect during the WebSocket handshake.
/// `refuse` carries the line the Room screen shows. The task must not follow that request.
public enum RoomRedirectDecision: Equatable, Sendable {
    case follow
    case refuse(String)
}

/// The room-memory WebSocket address: parse, the built-in default, and `UserDefaults`.
///
/// The built-in host is `CaptureRecipe.tailscaleDNS`, port 8767. `URLSession` resolves that
/// MagicDNS name only when Tailscale DNS is answering on this Mac. The SSH alias `spark` is not
/// used here; it resolves inside ssh, not for a socket. The tailnet IP is not a default and not
/// a fallback. A live screenshot that must show the IP sets `LABCOMMAND_ROOM_URL`.
///
/// `ws://` is cleartext. It is dialed only for a private address, Tailscale CGNAT (`100.64.0.0/10`),
/// or a `*.ts.net` host, and only when cleartext is permitted. The saved switch defaults to off.
/// `LABCOMMAND_ALLOW_CLEARTEXT=1` permits it for this process and is not saved.
/// `LABCOMMAND_ROOM_URL` chooses the address. It does not turn the switch on, on this launch or the next.
public enum RoomMemoryEndpoint {
    public static let defaultsKey = "roomMemoryEndpoint"
    public static let cleartextDefaultsKey = "roomMemoryAllowCleartextWebSocket"
    public static let environmentKey = "LABCOMMAND_ROOM_URL"
    /// Process-local opt-in. The only granting value is `1`. It is never written to the saved switch.
    public static let allowCleartextEnvironmentKey = "LABCOMMAND_ALLOW_CLEARTEXT"

    public static let cleartextWarning = "Warning: cleartext WebSocket sends room memory without encryption. ws:// is limited to private addresses, Tailscale CGNAT (100.64.0.0/10), and .ts.net hosts."
    public static let cleartextOffNote = "Cleartext is off. A ws:// address is not used until you turn this on. wss:// is always allowed."
    /// Shown beside the switch. The switch itself stays on the saved value.
    public static let cleartextEnvironmentNote = "LABCOMMAND_ALLOW_CLEARTEXT=1 allows cleartext for this launch. The switch shows the saved setting and does not turn this launch off."
    public static let downgradePrompt = "This replaces a secure wss:// address with cleartext ws://. Room memory will travel without encryption."

    /// `ws://<MagicDNS>:8767`. See the type discussion for when that name resolves.
    public static let defaultURL: URL = {
        let raw = "ws://\(CaptureRecipe.tailscaleDNS):8767"
        guard case .success(let url) = parse(raw) else {
            preconditionFailure("built-in room endpoint must parse")
        }
        return url
    }()

    /// `ws` or `wss`, a host, and an explicit port in `1...65535`. Empty path or `/` is accepted
    /// and dropped. Query, fragment, and userinfo are rejected. The scheme compare is
    /// case-insensitive. The returned URL has no trailing slash.
    public static func parse(_ raw: String) -> Result<URL, RoomMemoryEndpointError> {
        let trimmed = raw.trimmingCharacters(in: .whitespacesAndNewlines)
        if trimmed.isEmpty { return .failure(.empty) }
        guard let components = URLComponents(string: trimmed), components.scheme != nil else {
            return .failure(.unparseable)
        }
        guard let scheme = components.scheme?.lowercased(), scheme == "ws" || scheme == "wss" else {
            return .failure(.unsupportedScheme)
        }
        if components.user != nil || components.password != nil {
            return .failure(.userinfo)
        }
        guard let host = components.host, !host.isEmpty else {
            return .failure(.missingHost)
        }
        guard let port = components.port else {
            return .failure(.missingPort)
        }
        guard (1...65535).contains(port) else {
            return .failure(.badPort)
        }
        if components.path != "" && components.path != "/" {
            return .failure(.path)
        }
        if components.query != nil { return .failure(.query) }
        if components.fragment != nil { return .failure(.fragment) }
        let storedHost: String
        switch hostForDial(host) {
        case .failure(let error):
            return .failure(error)
        case .success(let strict):
            storedHost = strict
        }
        var canonical = URLComponents()
        canonical.scheme = scheme
        canonical.host = storedHost
        canonical.port = port
        guard let url = canonical.url else { return .failure(.unparseable) }
        return .success(url)
    }

    /// Private ranges (RFC1918, loopback, link-local), CGNAT `100.64.0.0/10`, or a `*.ts.net` name.
    /// An IPv4 host counts only as strict dotted decimal, classified as given: a trailing dot is
    /// not removed before that check. One trailing dot is stripped only for a `.ts.net` name.
    /// Public names and public addresses are false. `localhost` is false; use `127.0.0.1`.
    public static func allowsCleartextHost(_ host: String) -> Bool {
        var name = host.trimmingCharacters(in: .whitespacesAndNewlines).lowercased()
        if name.isEmpty || name.contains("%") { return false }
        if name.hasPrefix("[") && name.hasSuffix("]") {
            name = String(name.dropFirst().dropLast())
        }
        if name.contains(":") { return allowsCleartextIPv6(name) }
        // The address is classified as written. Stripping a trailing dot here would approve
        // 10.0.0.1. and a later dial would look that name up.
        if allowsCleartextIPv4(name) { return true }
        return allowsCleartextTailnetName(name)
    }

    /// One trailing dot is an absolute DNS name (`spark.ts.net.`). A second dot stays, so
    /// `spark.ts.net..` is not a tailnet name.
    private static func allowsCleartextTailnetName(_ name: String) -> Bool {
        let absolute = name.hasSuffix(".") ? String(name.dropLast()) : name
        guard absolute.hasSuffix(".ts.net") else { return false }
        let prefix = absolute.dropLast(".ts.net".count)
        return !prefix.isEmpty && !prefix.hasSuffix(".")
    }

    public static let cleartextDisabledPrefix = "Cleartext WebSocket is off."

    /// Why a fresh install does not connect, and what the operator can do. The default address is `ws://`
    /// and the switch starts off.
    public static func cleartextDisabledMessage(url: URL) -> String {
        "\(cleartextDisabledPrefix) \(url.absoluteString) is an unencrypted ws:// connection, so LabCommand will not connect. "
            + "Turn on Allow cleartext WebSocket in Settings, or enter a wss:// address."
    }

    /// True when `message` is the cleartext-off block. The Room page then offers one control that opens that switch.
    public static func offersCleartextSettings(_ message: String) -> Bool {
        message.hasPrefix(cleartextDisabledPrefix)
    }

    /// `1` opts this process in. Any other value, including a missing key, does not.
    /// This does not read `LABCOMMAND_ROOM_URL` and does not write the saved switch.
    public static func environmentAllowsCleartext(_ environment: [String: String]) -> Bool {
        environment[allowCleartextEnvironmentKey] == "1"
    }

    public static func cleartextHostMessage(host: String) -> String {
        "Cleartext ws:// is not allowed for \(host). Use a private address, Tailscale CGNAT (100.64.0.0/10), a .ts.net host, or wss://."
    }

    /// `wss://` is always allowed. `ws://` needs an allowed host and `allowCleartext`.
    public static func admit(_ url: URL, allowCleartext: Bool) -> RoomCleartextDecision {
        guard url.scheme?.lowercased() == "ws" else { return .allowed }
        let host = url.host ?? ""
        guard allowsCleartextHost(host) else {
            return .blocked(cleartextHostMessage(host: host))
        }
        guard allowCleartext else {
            return .blocked(cleartextDisabledMessage(url: url))
        }
        return .allowed
    }

    /// The handshake may follow `proposed` only when that destination would itself be dialed.
    /// A secure URL redirected to `ws://` or `http://` is refused: the operator did not confirm
    /// that downgrade. The saved switch is not confirmation of a server-chosen cleartext hop.
    /// `http` and `https` are judged as `ws` and `wss`. A destination `parse` rejects is refused.
    ///
    /// `from` is the URL that sent this redirect, or an earlier secure URL when that response does
    /// not show one. `RoomRedirectChain` keeps that memory across hops. `URLSession` leaves
    /// `originalRequest` on the first URL for the whole task, so a chain must not pass it every time.
    public static func decideRedirect(from original: URL, to proposed: URL, allowCleartext: Bool) -> RoomRedirectDecision {
        if isSecureScheme(original.scheme), isCleartextScheme(proposed.scheme) {
            return .refuse("Refused a redirect from \(original.absoluteString) to cleartext \(proposed.absoluteString).")
        }
        let destination = asWebSocket(proposed)
        switch parse(destination.absoluteString) {
        case .failure:
            return .refuse("Refused a redirect to \(proposed.absoluteString).")
        case .success(let url):
            switch admit(url, allowCleartext: allowCleartext) {
            case .allowed:
                return .follow
            case .blocked(let message):
                return .refuse("Refused a redirect to \(url.absoluteString). \(message)")
            }
        }
    }

    /// Process start only. A valid `LABCOMMAND_ROOM_URL` wins and is not written, except a `ws://`
    /// value does not replace a saved `wss://` address. Then a valid saved string. Then `defaultURL`.
    /// An invalid environment value or saved string is skipped. This does not apply the cleartext rule
    /// and does not grant cleartext. Dialing does, from the saved switch alone.
    public static func resolve(environment: [String: String], defaults: UserDefaults) -> URL {
        let saved = defaults.string(forKey: defaultsKey).flatMap { raw -> URL? in
            if case .success(let url) = parse(raw) { return url }
            return nil
        }
        if let raw = environment[environmentKey], case .success(let url) = parse(raw) {
            if url.scheme == "ws", saved?.scheme == "wss" {
                return saved ?? url
            }
            return url
        }
        if let saved { return saved }
        return defaultURL
    }

    /// Writes the canonical string. Callers pass a URL `parse` already accepted.
    public static func save(_ url: URL, to defaults: UserDefaults) {
        defaults.set(url.absoluteString, forKey: defaultsKey)
    }

    static func isSecureURL(_ url: URL) -> Bool {
        isSecureScheme(url.scheme)
    }

    private static func isSecureScheme(_ scheme: String?) -> Bool {
        switch scheme?.lowercased() {
        case "wss", "https": return true
        default: return false
        }
    }

    private static func isCleartextScheme(_ scheme: String?) -> Bool {
        switch scheme?.lowercased() {
        case "ws", "http": return true
        default: return false
        }
    }

    /// The handshake is HTTP. A redirect `Location` may say `http` or `https` for a WebSocket hop.
    private static func asWebSocket(_ url: URL) -> URL {
        guard let scheme = url.scheme?.lowercased(), scheme == "http" || scheme == "https" else { return url }
        guard var parts = URLComponents(url: url, resolvingAgainstBaseURL: false) else { return url }
        parts.scheme = scheme == "http" ? "ws" : "wss"
        return parts.url ?? url
    }

    /// The host stored on a parsed URL. A strict IPv4 literal is its canonical dotted form,
    /// which is the same string. A legacy literal, or a strict address with a trailing dot,
    /// is refused so nothing later dials a different name.
    private static func hostForDial(_ host: String) -> Result<String, RoomMemoryEndpointError> {
        if let strict = canonicalDottedIPv4(host) { return .success(strict) }
        if isLegacyIPv4Literal(host) || isStrictIPv4WithTrailingDot(host) {
            return .failure(.ambiguousAddress)
        }
        return .success(host)
    }

    /// `10.0.0.1.` is the strict address plus one trailing dot. `inet_pton` rejects the dot,
    /// and a dial looks the text up as a name instead of using 10.0.0.1.
    private static func isStrictIPv4WithTrailingDot(_ host: String) -> Bool {
        guard host.hasSuffix(".") else { return false }
        return canonicalDottedIPv4(String(host.dropLast())) != nil
    }

    /// `inet_pton(AF_INET)` accepts a leading zero as decimal on this OS (`010` → 10).
    /// `inet_aton` reads that same text as octal (`010` → 8), and `getaddrinfo` accepts
    /// hex, short forms, and a single integer. The only presentation those lookups share
    /// is the text `inet_ntop` writes back. A later dial of `host` is then the address
    /// that was checked.
    static func canonicalDottedIPv4(_ host: String) -> String? {
        var addr = in_addr()
        guard host.withCString({ inet_pton(AF_INET, $0, &addr) }) == 1 else { return nil }
        var buffer = [CChar](repeating: 0, count: Int(INET_ADDRSTRLEN))
        let written = buffer.withUnsafeMutableBufferPointer { raw -> UnsafePointer<CChar>? in
            guard let base = raw.baseAddress else { return nil }
            return inet_ntop(AF_INET, &addr, base, socklen_t(INET_ADDRSTRLEN))
        }
        guard written != nil else { return nil }
        let canonical = String(cString: buffer)
        guard canonical == host else { return nil }
        return canonical
    }

    /// Forms `inet_aton` would dial as a different address than the text: a leading zero,
    /// a short form (`10.1`), hex (`0x0a.0.0.1`), or a single integer (`167772161`).
    /// A strict dotted-decimal address is not legacy.
    private static func isLegacyIPv4Literal(_ host: String) -> Bool {
        if canonicalDottedIPv4(host) != nil { return false }
        let parts = host.lowercased().split(separator: ".", omittingEmptySubsequences: false)
        guard !parts.isEmpty, parts.allSatisfy({ !$0.isEmpty }) else { return false }
        return parts.allSatisfy { part in
            if part.allSatisfy({ $0.isASCII && $0.isNumber }) { return true }
            let digits = part.dropFirst(2)
            return part.hasPrefix("0x") && !digits.isEmpty && digits.allSatisfy(\.isHexDigit)
        }
    }

    private static func allowsCleartextIPv4(_ host: String) -> Bool {
        guard let canonical = canonicalDottedIPv4(host) else { return false }
        let parts = canonical.split(separator: ".")
        guard parts.count == 4, let a = Int(parts[0]), let b = Int(parts[1]) else { return false }
        if a == 10 { return true }
        if a == 127 { return true }
        if a == 192 && b == 168 { return true }
        if a == 172 && (16...31).contains(b) { return true }
        if a == 169 && b == 254 { return true }
        if a == 100 && (64...127).contains(b) { return true }
        return false
    }

    /// ULA `fc00::/7`, link-local `fe80::/10`, and `::1`. IPv4-mapped forms are rejected.
    private static func allowsCleartextIPv6(_ host: String) -> Bool {
        guard !host.contains("."), let bytes = ipv6Bytes(host), bytes.count == 16 else { return false }
        if bytes == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1] { return true }
        if bytes[0] & 0xfe == 0xfc { return true }
        if bytes[0] == 0xfe && (bytes[1] & 0xc0) == 0x80 { return true }
        return false
    }

    private static func ipv6Bytes(_ host: String) -> [UInt8]? {
        let pieces = host.components(separatedBy: "::")
        guard pieces.count <= 2 else { return nil }
        func groups(_ text: String) -> [UInt16]? {
            if text.isEmpty { return [] }
            var out: [UInt16] = []
            for group in text.split(separator: ":", omittingEmptySubsequences: false) {
                guard !group.isEmpty, group.count <= 4, let value = UInt16(group, radix: 16) else { return nil }
                out.append(value)
            }
            return out
        }
        let combined: [UInt16]
        if pieces.count == 1 {
            guard let whole = groups(pieces[0]), whole.count == 8 else { return nil }
            combined = whole
        } else {
            guard let left = groups(pieces[0]), let right = groups(pieces[1]) else { return nil }
            let missing = 8 - left.count - right.count
            guard missing >= 1 else { return nil }
            combined = left + Array(repeating: 0, count: missing) + right
        }
        guard combined.count == 8 else { return nil }
        var bytes: [UInt8] = []
        bytes.reserveCapacity(16)
        for group in combined {
            bytes.append(UInt8(group >> 8))
            bytes.append(UInt8(truncatingIfNeeded: group))
        }
        return bytes
    }
}

/// One WebSocket handshake's redirects.
///
/// `URLSession` reports each redirecting response as `HTTPURLResponse.url` and rewrites `ws`/`wss`
/// to `http`/`https`, but `originalRequest` stays the first URL. A secure scheme on the dialed URL,
/// on a redirecting response, or on a hop already followed makes a later `ws://` or `http://` a
/// downgrade. The saved cleartext switch does not confirm that downgrade.
public struct RoomRedirectChain: Equatable, Sendable {
    /// A secure URL already on this handshake.
    public private(set) var secureHop: URL?

    public init(dialed: URL?) {
        if let dialed, RoomMemoryEndpoint.isSecureURL(dialed) {
            secureHop = dialed
        }
    }

    /// `redirecting` is the URL that sent this redirect. Nil when the response has no URL.
    /// A cleartext redirecting URL does not erase a secure hop already recorded.
    public mutating func decide(redirecting: URL?, to proposed: URL?, allowCleartext: Bool) -> RoomRedirectDecision {
        guard let proposed else { return .refuse("Refused a redirect.") }
        if let redirecting, RoomMemoryEndpoint.isSecureURL(redirecting) {
            secureHop = redirecting
        }
        guard let from = secureHop ?? redirecting else {
            return .refuse("Refused a redirect to \(proposed.absoluteString).")
        }
        let decision = RoomMemoryEndpoint.decideRedirect(from: from, to: proposed, allowCleartext: allowCleartext)
        if case .follow = decision, RoomMemoryEndpoint.isSecureURL(proposed) {
            secureHop = proposed
        }
        return decision
    }
}

/// The one current room-memory endpoint for this process.
///
/// Created once from the environment and defaults. `commit` does not read the environment again,
/// so Settings can reopen onto the value the operator just saved while `LABCOMMAND_ROOM_URL` was set.
@MainActor
public final class RoomMemoryEndpointStore {
    public private(set) var current: URL
    /// The saved switch. Absent and explicit false are both off.
    /// `LABCOMMAND_ALLOW_CLEARTEXT=1` does not change this and is not written.
    public private(set) var allowCleartext: Bool
    /// This process was started with `LABCOMMAND_ALLOW_CLEARTEXT=1`. Never written.
    public let cleartextFromEnvironment: Bool
    private let defaults: UserDefaults

    /// The saved switch, or the process-local opt-in. Off when neither is set.
    /// An environment URL never turns this on.
    public var cleartextPermitted: Bool { allowCleartext || cleartextFromEnvironment }

    public init(environment: [String: String], defaults: UserDefaults) {
        self.defaults = defaults
        allowCleartext = defaults.bool(forKey: RoomMemoryEndpoint.cleartextDefaultsKey)
        cleartextFromEnvironment = RoomMemoryEndpoint.environmentAllowsCleartext(environment)
        current = RoomMemoryEndpoint.resolve(environment: environment, defaults: defaults)
    }

    public func setAllowCleartext(_ allow: Bool) {
        allowCleartext = allow
        defaults.set(allow, forKey: RoomMemoryEndpoint.cleartextDefaultsKey)
    }

    /// Validates `raw`. On success, persists it, updates `current`, and retargets `client` when
    /// one exists. On failure or when a `wss://` to `ws://` change still needs confirmation, the
    /// saved preference, `current`, and `client` stay as they were.
    public func commit(
        _ raw: String,
        retargeting client: LiveRoomClient?,
        confirmCleartextDowngrade: Bool = false
    ) -> RoomEndpointCommitResult {
        switch RoomMemoryEndpoint.parse(raw) {
        case .failure(let error):
            return .rejected(error.message)
        case .success(let url):
            if url.absoluteString == current.absoluteString { return .saved }
            if url.scheme == "ws" {
                switch RoomMemoryEndpoint.admit(url, allowCleartext: cleartextPermitted) {
                case .blocked(let message):
                    return .rejected(message)
                case .allowed:
                    break
                }
                if current.scheme?.lowercased() == "wss" && !confirmCleartextDowngrade {
                    return .needsCleartextConfirmation
                }
            }
            RoomMemoryEndpoint.save(url, to: defaults)
            current = url
            client?.retarget(url)
            return .saved
        }
    }
}
