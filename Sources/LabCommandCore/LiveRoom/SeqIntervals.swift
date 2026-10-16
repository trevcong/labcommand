import Foundation

/// A set of `UInt64` seqs as canonical inclusive ranges: sorted, disjoint and non-adjacent.
/// Pure value; the mirror's received coverage, the publisher's declared-unavailable seqs and the
/// ranges a `SyncEnd` names are all kept this way.
public struct IntervalSet: Sendable, Equatable {
    /// The canonical ranges, increasing.
    public private(set) var ranges: [ClosedRange<UInt64>] = []

    public init() {}

    /// Canonicalizes any ranges (overlapping or touching ones merge).
    public init<S: Sequence>(_ ranges: S) where S.Element == ClosedRange<UInt64> {
        for range in ranges { formUnion(range) }
    }

    public var isEmpty: Bool { ranges.isEmpty }
    public var max: UInt64? { ranges.last?.upperBound }

    /// The top of the run that starts at 0, or nil when 0 is not in the set.
    public var contiguousFromZeroThrough: UInt64? {
        guard let first = ranges.first, first.lowerBound == 0 else { return nil }
        return first.upperBound
    }

    /// Index of the first range whose upper bound is ≥ `value`.
    private func firstIndex(endingAtOrAfter value: UInt64) -> Int {
        var low = 0
        var high = ranges.count
        while low < high {
            let mid = (low + high) / 2
            if ranges[mid].upperBound < value { low = mid + 1 } else { high = mid }
        }
        return low
    }

    public func contains(_ value: UInt64) -> Bool {
        let index = firstIndex(endingAtOrAfter: value)
        return index < ranges.count && ranges[index].lowerBound <= value
    }

    /// Every seq of `range` is in the set.
    public func contains(_ range: ClosedRange<UInt64>) -> Bool {
        let index = firstIndex(endingAtOrAfter: range.lowerBound)
        return index < ranges.count && ranges[index].lowerBound <= range.lowerBound && ranges[index].upperBound >= range.upperBound
    }

    /// The number of ranges after inserting `value` (the cap checks this before inserting).
    public func rangeCount(inserting value: UInt64) -> Int {
        if contains(value) { return ranges.count }
        let index = firstIndex(endingAtOrAfter: value)
        let joinsNext = index < ranges.count && ranges[index].lowerBound == value &+ 1 && value != .max
        let joinsPrevious = index > 0 && ranges[index - 1].upperBound &+ 1 == value && value != 0
        switch (joinsPrevious, joinsNext) {
        case (true, true): return ranges.count - 1
        case (false, false): return ranges.count + 1
        default: return ranges.count
        }
    }

    /// Inserts one seq. Returns false when it was already there.
    @discardableResult
    public mutating func insert(_ value: UInt64) -> Bool {
        if contains(value) { return false }
        formUnion(value...value)
        return true
    }

    public mutating func formUnion(_ range: ClosedRange<UInt64>) {
        var low = range.lowerBound
        var high = range.upperBound
        // The first range that overlaps or touches `range` from below.
        let start = firstIndex(endingAtOrAfter: low == 0 ? 0 : low - 1)
        var end = start
        while end < ranges.count, high == .max || ranges[end].lowerBound <= high + 1 {
            low = Swift.min(low, ranges[end].lowerBound)
            high = Swift.max(high, ranges[end].upperBound)
            end += 1
        }
        ranges.replaceSubrange(start..<end, with: [low...high])
    }

    public mutating func formUnion(_ other: IntervalSet) {
        for range in other.ranges { formUnion(range) }
    }

    /// The seqs of `range` that are not in the set, as canonical ranges.
    public func gaps(in range: ClosedRange<UInt64>) -> IntervalSet {
        var out = IntervalSet()
        var next = range.lowerBound
        var done = false
        for held in ranges where held.upperBound >= range.lowerBound {
            if held.lowerBound > range.upperBound { break }
            if held.lowerBound > next {
                out.ranges.append(next...(held.lowerBound - 1))
            }
            if held.upperBound >= range.upperBound {
                done = true
                break
            }
            next = held.upperBound + 1
        }
        if !done, next <= range.upperBound {
            out.ranges.append(next...range.upperBound)
        }
        return out
    }

    public func subtracting(_ other: IntervalSet) -> IntervalSet {
        var out = IntervalSet()
        for range in ranges {
            out.ranges.append(contentsOf: other.gaps(in: range).ranges)
        }
        return out
    }

    public func intersection(_ other: IntervalSet) -> IntervalSet {
        var out = IntervalSet()
        var a = 0
        var b = 0
        while a < ranges.count, b < other.ranges.count {
            let low = Swift.max(ranges[a].lowerBound, other.ranges[b].lowerBound)
            let high = Swift.min(ranges[a].upperBound, other.ranges[b].upperBound)
            if low <= high { out.ranges.append(low...high) }
            if ranges[a].upperBound < other.ranges[b].upperBound { a += 1 } else { b += 1 }
        }
        return out
    }
}
