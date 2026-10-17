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
        case .unparseable: return "That is not a WebSocket URL."
        }
    }
}

/// The room-memory WebSocket address: parse, the built-in default, and `UserDefaults`.
///
/// The built-in host is `CaptureRecipe.tailscaleDNS`, port 8767. `URLSession` resolves that
/// MagicDNS name only when Tailscale DNS is answering on this Mac. The SSH alias `spark` is not
/// used here; it resolves inside ssh, not for a socket. The tailnet IP is not a default. A live
/// screenshot that must show the IP sets `LABCOMMAND_ROOM_URL`.
public enum RoomMemoryEndpoint {
    public static let defaultsKey = "roomMemoryEndpoint"
    public static let environmentKey = "LABCOMMAND_ROOM_URL"

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
        var canonical = URLComponents()
        canonical.scheme = scheme
        canonical.host = host
        canonical.port = port
        guard let url = canonical.url else { return .failure(.unparseable) }
        return .success(url)
    }

    /// Process start only. A valid `LABCOMMAND_ROOM_URL` wins and is not written. Then a valid
    /// saved string. Then `defaultURL`. An invalid environment value or saved string is skipped.
    public static func resolve(environment: [String: String], defaults: UserDefaults) -> URL {
        if let raw = environment[environmentKey], case .success(let url) = parse(raw) {
            return url
        }
        if let raw = defaults.string(forKey: defaultsKey), case .success(let url) = parse(raw) {
            return url
        }
        return defaultURL
    }

    /// Writes the canonical string. Callers pass a URL `parse` already accepted.
    public static func save(_ url: URL, to defaults: UserDefaults) {
        defaults.set(url.absoluteString, forKey: defaultsKey)
    }
}

/// The one current room-memory endpoint for this process.
///
/// Created once from the environment and defaults. `commit` does not read the environment again,
/// so Settings can reopen onto the value the operator just saved while `LABCOMMAND_ROOM_URL` was set.
@MainActor
public final class RoomMemoryEndpointStore {
    public private(set) var current: URL
    private let defaults: UserDefaults

    public init(environment: [String: String], defaults: UserDefaults) {
        self.defaults = defaults
        current = RoomMemoryEndpoint.resolve(environment: environment, defaults: defaults)
    }

    /// Validates `raw`. On success, persists it, updates `current`, and retargets `client` when
    /// one exists. On failure, the saved preference, `current`, and `client` stay as they were.
    /// Returns the operator-facing reason, or nil when the commit landed.
    public func commit(_ raw: String, retargeting client: LiveRoomClient?) -> String? {
        switch RoomMemoryEndpoint.parse(raw) {
        case .failure(let error):
            return error.message
        case .success(let url):
            RoomMemoryEndpoint.save(url, to: defaults)
            current = url
            client?.retarget(url)
            return nil
        }
    }
}
