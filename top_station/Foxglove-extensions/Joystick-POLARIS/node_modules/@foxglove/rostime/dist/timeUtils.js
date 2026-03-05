"use strict";
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/
//
// This file incorporates work covered by the following copyright and
// permission notice:
//
//   Copyright 2018-2021 Cruise LLC
//
//   This source code is licensed under the Apache License, Version 2.0,
//   found at http://www.apache.org/licenses/LICENSE-2.0
//   You may not use this file except in compliance with the License.
Object.defineProperty(exports, "__esModule", { value: true });
exports.areEqual = exports.isGreaterThan = exports.isLessThan = exports.compare = exports.isTimeInRangeInclusive = exports.clampTime = exports.fromMicros = exports.fromMillis = exports.toMillis = exports.fromNanoSec = exports.fromSec = exports.toSec = exports.toMicroSec = exports.toNanoSec = exports.subtract = exports.add = exports.fixTime = exports.interpolate = exports.percentOf = exports.fromDate = exports.toDate = exports.fromRFC3339String = exports.toRFC3339String = exports.fromString = exports.toString = exports.isTime = void 0;
/**
 * Test if a given object matches the signature of { sec: number; nsec: number }
 * @param obj Object to test
 * @returns True if the object is equivalent to a Time object, otherwise false
 */
function isTime(obj) {
    return (typeof obj === "object" &&
        // eslint-disable-next-line @typescript-eslint/strict-boolean-expressions
        !!obj &&
        "sec" in obj &&
        "nsec" in obj &&
        Object.getOwnPropertyNames(obj).length === 2);
}
exports.isTime = isTime;
/**
 * Converts a Time to a string containing a floating point number of seconds
 * @param stamp Time to convert
 * @param allowNegative Allow negative times to be converted
 * @returns String timestamp containing a floating point number of seconds
 */
function toString(stamp, allowNegative = false) {
    if (!allowNegative && (stamp.sec < 0 || stamp.nsec < 0)) {
        throw new Error(`Invalid negative time { sec: ${stamp.sec}, nsec: ${stamp.nsec} }`);
    }
    const sec = Math.floor(stamp.sec);
    const nsec = Math.floor(stamp.nsec);
    return `${sec}.${nsec.toFixed().padStart(9, "0")}`;
}
exports.toString = toString;
/**
 * Parse fractional seconds (digits following a decimal separator ".") and interpret them as an
 * integer number of nanoseconds. Because of the rounding behavior, this function may return 1e9 (a
 * value that would be too large for the `nsec` field).
 */
function parseNanoseconds(digits) {
    // There can be 9 digits of nanoseconds. If the fractional part is "1", we need to add eight
    // zeros. Also, make sure we round to an integer if we need to _remove_ digits.
    const digitsShort = 9 - digits.length;
    return Math.round(parseInt(digits, 10) * 10 ** digitsShort);
}
/**
 * Converts a string containing floating point number of seconds to a Time. We use a string because
 * nanosecond precision cannot be stored in a 64-bit float for large values (e.g. UNIX timestamps).
 * @param stamp UNIX timestamp containing a whole or floating point number of seconds
 * @returns Time object on success, undefined on failure
 */
function fromString(stamp) {
    if (/^\d+\.?$/.test(stamp)) {
        // Whole number with optional "." at the end.
        const sec = parseInt(stamp, 10);
        return { sec: isNaN(sec) ? 0 : sec, nsec: 0 };
    }
    if (!/^\d+\.\d+$/.test(stamp)) {
        // Not digits.digits -- invalid.
        return undefined;
    }
    const partials = stamp.split(".");
    if (partials.length === 0) {
        return undefined;
    }
    const [first, second] = partials;
    if (first == undefined || second == undefined) {
        return undefined;
    }
    // It's possible we rounded to { sec: 1, nsec: 1e9 }, which is invalid, so fixTime.
    const sec = parseInt(first, 10);
    const nsec = parseNanoseconds(second);
    return fixTime({ sec: isNaN(sec) ? 0 : sec, nsec });
}
exports.fromString = fromString;
/**
 * Converts a Time to a string compatible with RFC3339/ISO8601. Similar to
 * `toDate(stamp).toISOString()`, but with nanosecond precision.
 * @param stamp Time to convert
 */
function toRFC3339String(stamp) {
    if (stamp.sec < 0 || stamp.nsec < 0) {
        throw new Error(`Invalid negative time { sec: ${stamp.sec}, nsec: ${stamp.nsec} }`);
    }
    if (stamp.nsec >= 1e9) {
        throw new Error(`Invalid nanosecond value ${stamp.nsec}`);
    }
    const date = new Date(stamp.sec * 1000);
    const year = date.getUTCFullYear();
    const month = (date.getUTCMonth() + 1).toFixed().padStart(2, "0");
    const day = date.getUTCDate().toFixed().padStart(2, "0");
    const hour = date.getUTCHours().toFixed().padStart(2, "0");
    const minute = date.getUTCMinutes().toFixed().padStart(2, "0");
    const second = date.getUTCSeconds().toFixed().padStart(2, "0");
    const nanosecond = stamp.nsec.toFixed().padStart(9, "0");
    return `${year}-${month}-${day}T${hour}:${minute}:${second}.${nanosecond}Z`;
}
exports.toRFC3339String = toRFC3339String;
/**
 * Parses a Time from a string compatible with a subset of ISO8601/RFC3339. Similar to
 * `fromDate(new Date(string))`, but with nanosecond precision.
 * @param stamp Time to convert
 */
function fromRFC3339String(stamp) {
    const match = /^(\d{4,})-(\d\d)-(\d\d)[Tt](\d\d):(\d\d):(\d\d)(?:\.(\d+))?(?:[Zz]|([+-])(\d\d):(\d\d))$/.exec(stamp);
    if (match == null) {
        return undefined;
    }
    const [, year, month, day, hour, minute, second, frac, plusMinus, offHours, offMinutes] = match;
    const offSign = plusMinus === "-" ? -1 : 1;
    const utcMillis = Date.UTC(+year, +month - 1, +day, +hour - offSign * +(offHours ?? 0), +minute - offSign * +(offMinutes ?? 0), +second);
    if (utcMillis % 1000 !== 0) {
        return undefined;
    }
    // It's possible we rounded to { sec: 1, nsec: 1e9 }, which is invalid, so fixTime.
    return fixTime({
        sec: utcMillis / 1000,
        nsec: frac != undefined ? parseNanoseconds(frac) : 0,
    });
}
exports.fromRFC3339String = fromRFC3339String;
/**
 * Convert a Time to a JavaScript Date object. NOTE: sub-millisecond precision is lost.
 * @param stamp Time to convert
 * @returns Date representing the given Time as accurately as it can
 */
function toDate(stamp) {
    const { sec, nsec } = stamp;
    return new Date(sec * 1000 + nsec / 1e6);
}
exports.toDate = toDate;
/**
 * Conver a JavaScript Date object to a Time.
 * @param date Date to convert
 * @returns Time representing the given Date
 */
function fromDate(date) {
    const millis = date.getTime();
    const remainder = millis % 1000;
    return { sec: Math.floor(millis / 1000), nsec: remainder * 1e6 };
}
exports.fromDate = fromDate;
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
function percentOf(start, end, target) {
    const totalDuration = subtract(end, start);
    const targetDuration = subtract(target, start);
    return toSec(targetDuration) / toSec(totalDuration);
}
exports.percentOf = percentOf;
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
function interpolate(start, end, fraction) {
    const duration = subtract(end, start);
    return add(start, fromSec(fraction * toSec(duration)));
}
exports.interpolate = interpolate;
/**
 * Equivalent to fromNanoSec(toNanoSec(t)), but no chance of precision loss. nsec should be
 * non-negative, and less than 1e9.
 * @param t Potentially un-normalized time with the nsec (nanoseconds) value containing a value
 *   higher than one second (1e9)
 * @param allowNegative Allow negative times to be normalized
 * @returns A normalized Time
 */
function fixTime(t, allowNegative = false) {
    const durationNanos = t.nsec;
    const secsFromNanos = Math.floor(durationNanos / 1e9);
    const newSecs = t.sec + secsFromNanos;
    const remainingDurationNanos = durationNanos % 1e9;
    // use Math.abs here to prevent -0 when there is exactly 1 second of negative nanoseconds passed in
    const newNanos = Math.abs(Math.sign(remainingDurationNanos) === -1
        ? 1e9 + remainingDurationNanos
        : remainingDurationNanos);
    const result = { sec: newSecs, nsec: newNanos };
    if ((!allowNegative && result.sec < 0) || result.nsec < 0) {
        throw new Error(`Cannot normalize invalid time ${toString(result, true)}`);
    }
    return result;
}
exports.fixTime = fixTime;
/**
 * Add two Times together
 * @param param0 First Time
 * @param param1 Second Time
 * @returns A normalized representation of the two Time objects added together
 */
function add({ sec: sec1, nsec: nsec1 }, { sec: sec2, nsec: nsec2 }) {
    return fixTime({ sec: sec1 + sec2, nsec: nsec1 + nsec2 });
}
exports.add = add;
/**
 * Subtract one Time from another
 * @param param0 First Time
 * @param param1 Time to subtract from the first Time
 * @returns A normalized representation of the second Time subtracted from the first
 */
function subtract({ sec: sec1, nsec: nsec1 }, { sec: sec2, nsec: nsec2 }) {
    return fixTime({ sec: sec1 - sec2, nsec: nsec1 - nsec2 }, true);
}
exports.subtract = subtract;
/**
 * Convert Time to an integer number of nanoseconds
 * @param param0 Time to convert
 * @returns A bigint integer number of nanoseconds
 */
function toNanoSec({ sec, nsec }) {
    return BigInt(sec) * 1000000000n + BigInt(nsec);
}
exports.toNanoSec = toNanoSec;
/**
 * Convert Time to a floating point number of microseconds
 * @param param0 Time to convert
 * @returns A floating point number of microseconds
 */
function toMicroSec({ sec, nsec }) {
    return (sec * 1e9 + nsec) / 1000;
}
exports.toMicroSec = toMicroSec;
/**
 * Convert Time to a floating point number of seconds
 * @param param0 Time to convert
 * @returns A floating point number of seconds
 */
function toSec({ sec, nsec }) {
    return sec + nsec * 1e-9;
}
exports.toSec = toSec;
/**
 * Convert a floating point number of seconds to Time
 * @param value Number of seconds
 * @returns Time object
 */
function fromSec(value) {
    // From https://github.com/ros/roscpp_core/blob/indigo-devel/rostime/include/ros/time.h#L153
    let sec = Math.trunc(value);
    let nsec = Math.round((value - sec) * 1e9);
    sec += Math.trunc(nsec / 1e9);
    nsec %= 1e9;
    return { sec, nsec };
}
exports.fromSec = fromSec;
/**
 * Convert an integer number of nanoseconds to Time
 * @param nsec Nanoseconds integer
 * @returns Time object
 */
function fromNanoSec(nsec) {
    // From https://github.com/ros/roscpp_core/blob/86720717c0e1200234cc0a3545a255b60fb541ec/rostime/include/ros/impl/time.h#L63
    // and https://github.com/ros/roscpp_core/blob/7583b7d38c6e1c2e8623f6d98559c483f7a64c83/rostime/src/time.cpp#L536
    //
    // Note: BigInt(1e9) is slower than writing out the number
    return { sec: Number(nsec / 1000000000n), nsec: Number(nsec % 1000000000n) };
}
exports.fromNanoSec = fromNanoSec;
/**
 * Convert Time to an integer number of milliseconds
 * @param time Time to convert
 * @param roundUp Round up to nearest millisecond if true, otherwise round down. Defaults to true
 * @returns Integer number of milliseconds
 */
function toMillis(time, roundUp = true) {
    const secondsMillis = time.sec * 1e3;
    const nsecMillis = time.nsec / 1e6;
    return roundUp ? secondsMillis + Math.ceil(nsecMillis) : secondsMillis + Math.floor(nsecMillis);
}
exports.toMillis = toMillis;
/**
 * Convert milliseconds to Time
 * @param value Milliseconds number
 * @returns Time object
 */
function fromMillis(value) {
    let sec = Math.trunc(value / 1000);
    let nsec = Math.round((value - sec * 1000) * 1e6);
    sec += Math.trunc(nsec / 1e9);
    nsec %= 1e9;
    return { sec, nsec };
}
exports.fromMillis = fromMillis;
/**
 * Convert microseconds to Time
 * @param value Microseconds number
 * @returns Time object
 */
function fromMicros(value) {
    let sec = Math.trunc(value / 1e6);
    let nsec = Math.round((value - sec * 1e6) * 1e3);
    sec += Math.trunc(nsec / 1e9);
    nsec %= 1e9;
    return { sec, nsec };
}
exports.fromMicros = fromMicros;
/**
 * Clamp a given time value in the range from start to end (inclusive)
 * @param time Time to clamp
 * @param start Start of the target range
 * @param end End of the target range
 * @returns Clamped Time
 */
function clampTime(time, start, end) {
    if (compare(start, time) > 0) {
        return { sec: start.sec, nsec: start.nsec };
    }
    if (compare(end, time) < 0) {
        return { sec: end.sec, nsec: end.nsec };
    }
    return { sec: time.sec, nsec: time.nsec };
}
exports.clampTime = clampTime;
/**
 * Test if a given time is inside a test range
 * @param time Time to test
 * @param start Start of the test range
 * @param end End of the test range
 * @returns True if time falls in between start and end (inclusive)
 */
function isTimeInRangeInclusive(time, start, end) {
    if (compare(start, time) > 0 || compare(end, time) < 0) {
        return false;
    }
    return true;
}
exports.isTimeInRangeInclusive = isTimeInRangeInclusive;
/**
 * Comparison function for Time object that can be used for sorting
 * @param left First Time to compare
 * @param right Second Time to compare
 * @returns A positive value if left is larger than right, a negative value if right is larger than
 *   left, or zero if both times are equal
 */
function compare(left, right) {
    const secDiff = left.sec - right.sec;
    return secDiff !== 0 ? secDiff : left.nsec - right.nsec;
}
exports.compare = compare;
/**
 * Returns true if the left time is less than the right time, otherwise false
 * @param left Left side of comparison
 * @param right Right side of comparison
 * @returns Comparison result
 */
function isLessThan(left, right) {
    return compare(left, right) < 0;
}
exports.isLessThan = isLessThan;
/**
 * Returns true if the left time is greater than the right time, otherwise false
 * @param left Left side of the comparison
 * @param right Right side of the comparison
 * @returns Comparison result
 */
function isGreaterThan(left, right) {
    return compare(left, right) > 0;
}
exports.isGreaterThan = isGreaterThan;
/**
 * Returns true if both times have the same number of seconds and nanoseconds
 * @param left Left side of the comparison
 * @param right Right side of the comparison
 * @returns Equality result
 */
function areEqual(left, right) {
    return left.sec === right.sec && left.nsec === right.nsec;
}
exports.areEqual = areEqual;
//# sourceMappingURL=timeUtils.js.map