import { GeodeticCoordinates } from "./dropkick-reader";

/** WGS84 equatorial semi-axis "a" (m). */
const WGS84_MAJOR	 = 6378137.0

/** WGS84 polar semi-axis "b" (m). */
const WGS84_MINOR =	6356752.3142

/** Eccentricity = sqrt(1 - (b/a)^2). */
const WGS84_ECCENTRICITY =	0.081819190928906199466

/** Eccentricity squared. */
const WGS84_ECCENTRICITY_SQR =	0.006694380004260806515

const PI_2 = Math.PI / 2.0;

function DEGtoRAD(a: number) {
    return (a * Math.PI / 180.0);
}

function RADtoDEG(a: number) {
    return (a * 180.0 / Math.PI);
}

function normalizeLatitude(a:number):number {
	while (a > PI_2) {
		a -= PI_2
	}
	while (a < -PI_2) {
		a += PI_2
	}
	return a;
}

function normalizeLongitude(a:number):number {
	while (a > Math.PI) {
		a -= Math.PI
	}
	while (a < -Math.PI) {
		a += Math.PI
	}
	return a;
}

/*
 * Borrowed from my original ACM flight simulator source code (globe.c)
 *
 *  In the DIS 2.0 coordinate system:
 *
 *      positive Z axis is North;
 *      positive X axis points to 0N, 0E;
 *      positive Y axis points to 0N 90E.
 *
 *  So, North latitudes are positive; East longitudes are positive.
 *
 *  The world is considered a perfect ellipsoid based on the WGS84
 *  standard -- no correction is made to take into account height differences
 *  between the ellpsoid and the geoid.
 *
 *  "The Surveying Handbook", edited by Brinker and Minnick contains a decent
 *  discussion of the technical issues required to understand what's
 *  going on in this code.
 */

/**
 * Compute a location, given a course and distance from a starting location
 * 
 * @param p starting location
 * @param trueCourse_deg direction of movement (degrees)
 * @param d_meters distance to move (meters)
 * @returns new location
 */

export function traverseEllipsoid(
	p: GeodeticCoordinates, 
	trueCourse_deg:number, 
	d_meters: number): GeodeticCoordinates {

	const course_rad = DEGtoRAD(trueCourse_deg);
	return traverse (p, Math.sin(course_rad), Math.cos(course_rad), d_meters);
}

function traverse(p: GeodeticCoordinates,
	cos_course: number, 
	sin_course: number, 
	d_meters: number): GeodeticCoordinates
{
    let    res: GeodeticCoordinates = { lat_deg: 0, lon_deg: 0, alt_m: 0};
	let    n1, n2, m1;
	let    sin_lat, sin_lat_sqr, tan_lat, sin_course_sqr;
	let    delta_latitude, delta_longitude, d_sqr, cos_lat;
	let    B, C, /* D, */ E, h, sin_newlat;

/*  Increase our height to the height above the WGS-84 reference ellipsoid */

	let    wgs84_a = WGS84_MAJOR + p.alt_m;

	sin_lat = Math.sin(DEGtoRAD(p.lat_deg));
	sin_lat_sqr = sin_lat * sin_lat;
	cos_lat = Math.cos(DEGtoRAD(p.lat_deg));
	tan_lat = sin_lat / cos_lat;
	sin_course_sqr = sin_course * sin_course;
	d_sqr = d_meters * d_meters;

	n1 = wgs84_a / Math.sqrt(1.0 - WGS84_ECCENTRICITY_SQR * sin_lat_sqr);
	m1 = (wgs84_a * (1.0 - WGS84_ECCENTRICITY_SQR)) /
		Math.pow(1.0 - WGS84_ECCENTRICITY_SQR * sin_lat_sqr, 1.5);

	B = 1.0 / m1;

	h = d_meters * B * cos_course;

	C = tan_lat / (2.0 * m1 * n1);

	E = (1.0 + 3.0 * tan_lat * tan_lat) *
		(1.0 - WGS84_ECCENTRICITY_SQR * sin_lat_sqr) / (6.0 * wgs84_a * wgs84_a);

	delta_latitude = d_meters * B * cos_course -
		d_sqr * C * sin_course_sqr -
		h * d_sqr * E * sin_course_sqr;

	res.lat_deg = RADtoDEG(normalizeLatitude(DEGtoRAD(p.lat_deg) + delta_latitude))

	sin_newlat = Math.sin(DEGtoRAD(res.lat_deg));

	n2 = wgs84_a / Math.sqrt(1.0 - WGS84_ECCENTRICITY_SQR * sin_newlat * sin_newlat);

	delta_longitude = (d_meters * sin_course) / (n2 * Math.cos(DEGtoRAD(p.lat_deg)));

	res.lon_deg = RADtoDEG(normalizeLongitude(DEGtoRAD(p.lon_deg) + delta_longitude))

    return res;
}