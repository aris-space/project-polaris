import { Time } from "./Time";
/**
 * Test if a given object matches the signature of { sec: number; nsec: number }
 * @param obj Object to test
 * @returns True if the object is equivalent to a Time object, otherwise false
 */
export declare function isTime(obj?: unknown): obj is Time;
/**
 * Converts a Time to a string containing a floating point number of seconds
 * @param stamp Time to convert
 * @param allowNegative Allow negative times to be converted
 * @returns String timestamp containing a floating point number of seconds
 */
export declare function toString(stamp: Time, allowNegative?: boolean): string;
/**
 * Converts a string containing floating point number of seconds to a Time. We use a string because
 * nanosecond precision cannot be stored in a 64-bit float for large values (e.g. UNIX timestamps).
 * @param stamp UNIX timestamp containing a whole or floating point number of seconds
 * @returns Time object on success, undefined on failure
 */
export declare function fromString(stamp: string): Time | undefined;
/**
 * Converts a Time to a string compatible with RFC3339/ISO8601. Similar to
 * `toDate(stamp).toISOString()`, but with nanosecond precision.
 * @param stamp Time to convert
 */
export declare function toRFC3339String(stamp: Time): string;
/**
 * Parses a Time from a string compatible with a subset of ISO8601/RFC3339. Similar to
 * `fromDate(new Date(string))`, but with nanosecond precision.
 * @param stamp Time to convert
 */
export declare function fromRFC3339String(stamp: string): Time | undefined;
/**
 * Convert a Time to a JavaScript Date object. NOTE: sub-millisecond precision is lost.
 * @param stamp Time to convert
 * @returns Date representing the given Time as accurately as it can
 */
export declare function toDate(stamp: Time): Date;
/**
 * Conver a JavaScript Date object to a Time.
 * @param date Date to convert
 * @returns Time representing the given Date
 */
export declare function fromDate(date: Date): Time;
/**
 * Returns the fraction representing target's position in the range between start and end.
 * e.g. start = { sec: 0 }, end = { sec: 10 }, target = { sec: 5 } = 0.5
 * This is the reverse of the `interpolate()` method
 * @param start Start timestamp of the interpolation range
 * @param end End timestamp of the interpolation range
 * @param target Timestamp that will be measured relative to the interpolation range
 * @returns If target falls in between start and end (inclusive), it will be in the range [0.0-1.0].
 *   Otherwise, it is unbounded
 */
export declare function percentOf(start: Time, end: Time, target: Time): number;
/**
 * Linearly interpolate the range between start and end by a given fraction.
 * e.g. start = { sec: 0 }, end = { sec: 10 }, fraction = 0.5 = { sec: 5 }
 * This is the reverse of the `percentOf` method
 * @param start Start timestamp of the interpolation range
 * @param end End timestamp of the interpolation range
 * @param fraction Percent to interpolate along the range
 * @returns If fraction is in the range [0.0-1.0], the target will fall in between start and end\
 *   (inclusive). Otherwise, it is unbounded
 */
export declare function interpolate(start: Time, end: Time, fraction: number): Time;
/**
 * Equivalent to fromNanoSec(toNanoSec(t)), but no chance of precision loss. nsec should be
 * non-negative, and less than 1e9.
 * @param t Potentially un-normalized time with the nsec (nanoseconds) value containing a value
 *   higher than one second (1e9)
 * @param allowNegative Allow negative times to be normalized
 * @returns A normalized Time
 */
export declare function fixTime(t: Time, allowNegative?: boolean): Time;
/**
 * Add two Times together
 * @param param0 First Time
 * @param param1 Second Time
 * @returns A normalized representation of the two Time objects added together
 */
export declare function add({ sec: sec1, nsec: nsec1 }: Time, { sec: sec2, nsec: nsec2 }: Time): Time;
/**
 * Subtract one Time from another
 * @param param0 First Time
 * @param param1 Time to subtract from the first Time
 * @returns A normalized representation of the second Time subtracted from the first
 */
export declare function subtract({ sec: sec1, nsec: nsec1 }: Time, { sec: sec2, nsec: nsec2 }: Time): Time;
/**
 * Convert Time to an integer number of nanoseconds
 * @param param0 Time to convert
 * @returns A bigint integer number of nanoseconds
 */
export declare function toNanoSec({ sec, nsec }: Time): bigint;
/**
 * Convert Time to a floating point number of microseconds
 * @param param0 Time to convert
 * @returns A floating point number of microseconds
 */
export declare function toMicroSec({ sec, nsec }: Time): number;
/**
 * Convert Time to a floating point number of seconds
 * @param param0 Time to convert
 * @returns A floating point number of seconds
 */
export declare function toSec({ sec, nsec }: Time): number;
/**
 * Convert a floating point number of seconds to Time
 * @param value Number of seconds
 * @returns Time object
 */
export declare function fromSec(value: number): Time;
/**
 * Convert an integer number of nanoseconds to Time
 * @param nsec Nanoseconds integer
 * @returns Time object
 */
export declare function fromNanoSec(nsec: bigint): Time;
/**
 * Convert Time to an integer number of milliseconds
 * @param time Time to convert
 * @param roundUp Round up to nearest millisecond if true, otherwise round down. Defaults to true
 * @returns Integer number of milliseconds
 */
export declare function toMillis(time: Time, roundUp?: boolean): number;
/**
 * Convert milliseconds to Time
 * @param value Milliseconds number
 * @returns Time object
 */
export declare function fromMillis(value: number): Time;
/**
 * Convert microseconds to Time
 * @param value Microseconds number
 * @returns Time object
 */
export declare function fromMicros(value: number): Time;
/**
 * Clamp a given time value in the range from start to end (inclusive)
 * @param time Time to clamp
 * @param start Start of the target range
 * @param end End of the target range
 * @returns Clamped Time
 */
export declare function clampTime(time: Time, start: Time, end: Time): Time;
/**
 * Test if a given time is inside a test range
 * @param time Time to test
 * @param start Start of the test range
 * @param end End of the test range
 * @returns True if time falls in between start and end (inclusive)
 */
export declare function isTimeInRangeInclusive(time: Time, start: Time, end: Time): boolean;
/**
 * Comparison function for Time object that can be used for sorting
 * @param left First Time to compare
 * @param right Second Time to compare
 * @returns A positive value if left is larger than right, a negative value if right is larger than
 *   left, or zero if both times are equal
 */
export declare function compare(left: Time, right: Time): number;
/**
 * Returns true if the left time is less than the right time, otherwise false
 * @param left Left side of comparison
 * @param right Right side of comparison
 * @returns Comparison result
 */
export declare function isLessThan(left: Time, right: Time): boolean;
/**
 * Returns true if the left time is greater than the right time, otherwise false
 * @param left Left side of the comparison
 * @param right Right side of the comparison
 * @returns Comparison result
 */
export declare function isGreaterThan(left: Time, right: Time): boolean;
/**
 * Returns true if both times have the same number of seconds and nanoseconds
 * @param left Left side of the comparison
 * @param right Right side of the comparison
 * @returns Equality result
 */
export declare function areEqual(left: Time, right: Time): boolean;
//# sourceMappingURL=timeUtils.d.ts.map