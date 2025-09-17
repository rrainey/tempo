# Jump Analysis: Coordinate Systems and Plots

Analysis of different elements of a skydive will require us to plat 3D positions of skydivers.  Seometimes we will be plotting the world location of the jumper. In some cases, we will plot the positions of jumpers relative to each other.  In all these cases, we will need well-defined coordinate systems to perform the plotting and analysis. In this document we will formally describe several coordinate systems.  We will use each for different purposes related to storage, analysis, and rendering within the application.

The systems described here start with several coordinate frames commonly used in aerospace modeling and simulation.  These are formally defined in ANSI/AIAA R-004-1992, "Atmospheric and Space Flight Coordinate Systems".  We also define several more here which will be convenient for analysis of skydiving performance.  Because of its strong connection to GPS satellite use, we will also rely on the World Geodetic System, WGS-84 standard for expressing latitude, longitude and altitude coordinates.

## Definitions

**Fall Rate** and **Vertical Velocity** are synonymous terms.  Where stored, it should be expressed as **meters per second**. When printed, these are expressed as **miles per hour**.

**Ground Track** the compass angle of a path inscribed along the ground, relative to True North.  It should be stored as radians, and printed as degrees.

## Earth Centered Inertial (ECI, meters)

Earth-centered inertial (ECI) coordinate frames having their origin at the center of mass of Earth and the axes fixed with respect to the stars. Z-North pole, X-axis towards the Vernal Equinox.

## Earth Centered, Earth Fixed (ECEF, meters)

Our Earth Centered, Earth Fixed (ECEF or just ECF) frame has its Z-Axis out the north pole, X-Axis through 0-deg N/0-deg E and Y-axis forming a right handed coordinate system (0 N, 90 deg E).

## North, East, Down (NED, meters)

This coordinate frames are centered at sea-level for a given WGS-84 geodetic latitude and longitude. The coordinate frame has the origin at 0 meters, Mean Sea Level. The X-axis point North, the Z-axis is directed towards the local "Down" gravity vector, and the Y-axis completes the right-handed 3D coordinate system.

We'll talk more about this later, but we'll use the idealized WGS-84 ellipsoid (using WGS-84 major and minor axis lengths) to define Mean Seal Level at all locations.

## Jump Run Frame

It is tempting to call this the Exit Frame, but that would conflict with the Big Way FS term so just, "Jump Run Frame".  This is a dynamic NED frame 0 meters, MSL below the estimated jump exit lat/lon (for our purposes the time an location of exit will be set by the exit of the base (or solo) jumper).  The X-axis is rotated from north to align with the ground track of the (primary) jump aircraft.

## Base Exit Frame

This is a dynamic variant of a NED frame, centered on the real time position of the base jumper in a formation, rotated so that the X-axis is rotated to be aligned with the jump run ground track (as with the Jump Run Frame).

## Base Relative Frame

This is a dynamic variant of a NED frame, whose origin is centered on the real time position of the base jumper in a formation, rotated so that the X-axis is parallel to their spine.

## What -- or Who -- is the Base?

For our initial analytic investigation, the base will be a single designated person.  The base will be designated at the start of the analysis phase and, potentially, can be changed to allow for the analysis to be updated. 

Since the center of the Base in a Big Way Skydiving formation might be some distance away from a "base jumper" or "LO" - we expect that later versions of this software would support pre-designating that the Base might be "X meters in front" of a designated individual. That is an area for further study before committing to an approach (or defining the data elements required to support it).

## Conversion between coordinate systems

We anticipate the need to convert from one frame to another during analysis.  However, we do not expect to be required to develop conversion algorithms from a source frame to every other possible frame.  The frame conversion capabilities that we anticipate are listed in the table below:


| Source Frame    | WGS-84 Geodetic   | ECI      | ECF      | NED   | Base Exit   | Base Relative |
|----------------:|:-----------------:|:--------:|:--------:|:------:|:----------:|:-------------:|
| WGS-84 Geodetic |    N/A            |  -       |   -      |  Yes   |     Yes    |    Yes        |
| ECI             |    -              |   N/A    |  -       |  -      |    -        |   -            |
| ECF             |    -              |   -      | N/A      |   -     |    -        |   -            |
| NED             |    -              |   -       |  -       |  N/A   |    -        |   -            |
| Base Exit       |    -              |    -      |  -        |  -      |   N/A      |    -           |
| Base Relative   |    -              |   -       |  -        |  -      |   -         |     N/A       |  

In fact, for the analysis that we expect to perform within this application, we do not expect to use the ECI frame at all. And we probably need the ECF frame only insofar as it might be a useful intermediate point in other conversions.

## WGS-84,  altitudes: Geoid versus Idealized Ellipsoid

In our use of WGS-84 coordinates, we will have a Latitude (radians, North positive), Longitude (radians, east positive), and z (altitude, meters relative to Mean Sea Level, positive up)

For our purposes in this first round of analysis algorithm development, we will use the ideal ellipsoid definition of Mean Sea Level.  At some later date, we may introduce local Geoid (local gravity) corrections, which are location dependent.


## Analysis Scenario 1: Solo Jump Analysis

The path of the skydiver will be converted from the source GNSS (and barometric data for altitude) to the NED frame, centered at a fixed location on the airport grounds (nominally, the center of the landing zone). It can then be rendered using any one of several common 3D views (e.g., top-down). We will call this the NED,DZ frame.

Solo Jump analysis would also include several other standard time series plots: barometric altitude vs. time, G-forces vs. time, vertical velocity vs. time.  It also should include the ability to determine the range of vertical velocity of the skydiver - the analysis would normally be limited to a period after terminal velocity is achieved (say, 14 seconds) and estimated deployment.  The minima, maxima, and average could then be plotted against a curated baseline to give the skydiver a feel of whether they tended to be "fast or floaty" on that jump relative to others.

### Calibrated Fall Rate

Air density decreases with altitude. The ICAO Standard Atmosphere [citation needed] defines this expected decrease in density for "Standard Day" conditions.  This density lapse rate is also affected by temperature and humidity, although we will neglect those in this first pass analysis algorithms.  A difference in density will affect the terminal velocity of a skydiver.  In short, they will fall gradually more slowly at lower altitudes. This change will affect a group of skydivers who are close together in the same way, so they don't really notice it - but it will affect any numerical analysis of a jump, so we should be taking it into account.

Back of napkin calculations show that a jumper whose terminal velocity is 120 mph at 7,000 feet MSL, would fall at closer to 135 mph at 14,000 feet and 110mph near deployment altitude.  This is a significant variation - it should be accounted for our comparisons.  We propose to design a standard fall rate calibration factor function. The function will be based on the Standard Day altitude, MSL of the jumper. This factor will chosen be 1.0 at 7,000 feet MSL. Based on our back-of-napkin math, it will then vary below 1 above that altitude and > 1.0 below it.  This calibration factor can then be looked up for any any altitude in the jump. It can then be multiplied by the current vertical velocity.

Why is that calibration interesting?  If we can analyze fall rate over the course of a jump and factor out velocity differences caused by changing air density, we can reliably know how the jumper's inputs were contributing to fall rate results.

Standard Day Conditions

This table shows the Standard Day variation of air density by altitude.  It also shows the effect of air density variation on fall rate of a hypothetical skydiver whose weight and drag would have them falling at 120 mph at 7,000 feet.

| Altitude (ft, MSL) | Air density ρ (kg/m³) | Notional fall rate (mph) | Fall Rate Calibration Factor |
|---------------|------------------------|-------------------------|------------------------------|
| 20000         | 0.6527                 | 148.0                   | 0.8107                       |
| 18000         | 0.6982                 | 143.1                   | 0.8385                       |
| 16000         | 0.7460                 | 138.5                   | 0.8667                       |
| 14000         | 0.7963                 | 134.0                   | 0.8955                       |
| 12000         | 0.8491                 | 129.8                   | 0.9247                       |
| 10000         | 0.9046                 | 125.7                   | 0.9545                       |
| 9000          | 0.9334                 | 123.8                   | 0.9695                       |
| 8000          | 0.9629                 | 121.9                   | 0.9847                       |
| 7000          | 0.9930                 | 120.0   (reference)     | 1.0000                       |
| 6000          | 1.0239                 | 118.2                   | 1.0154                       |
| 5000          | 1.0555                 | 116.4                   | 1.0310                       |
| 4000          | 1.0879                 | 114.6                   | 1.0467                       |
| 3000          | 1.1210                 | 112.9                   | 1.0625                       |
| 2000          | 1.1549                 | 111.3                   | 1.0784                       |
| 1000          | 1.1896                 | 109.6                   | 1.0945                       |
| 0             | 1.2250                 | 108.0                   | 1.1107          

All of the vertical velocity plots previously mentioned shall include the ability to switch between plotting calibrated and absolute (uncalibrated) vertical velocities/fall rates. Calibrated velocity shall be the default.

## Analysis Scenario 2: Formation Skydiving Jump Analysis

The analyst will interactively designate one jumper from the group of collected logs as the "Base".  GNSS (and barometric) position for all jumpers can be transformed into individual NED,DZ frame coordinate sets.  Once the Base is designated, the individual NED,DZ sets for each jumper can be converted to the Base Exit frame.  This can then be rendered as a moving plot, interactively depicting the position of each skydiver in their position relative to the base skydiver.

## Implementation

### Reference Locations (Dropzones)

Several of these coordinate frames will be based on a static reference point: the center of the drop zone or center of a desired landing area.  It follows that we should maintain a table of these locations by name.  In the user's profile, a user shall be able to select the default reference location for their jumps, and also be able to override that for each jump.  Likewise, Formation Skydives should have an associated reference location that can be selected by the analyst. This FS value overrides any user setting for the scope of group analysis.

- DZ / Target Name - "Spaceland Dallas, D Landing Area"
- Latitude (WGS-84; double-precision, stored as radians, printed as decimal degrees)
- Longitude (WGS-84; double-precision, stored as radians, printed as decimal degrees)
- Altitude (stored as meters,MSL - printed as feet,MSL)

Reference Locations are defined and maintained the System Administrators.  In the Home Page UI for administrators, it will appear as a "Dropzones" item on the left-hand navigation bar just below "Devices".

### Existing code re-use

Tempo-insights application will use these modules for coordinate and unit conversion wherever practical:

- **dropkick-reader.ts** - a Log file reader adapted to process Tempo-BT log files.  This module also defines a number of Typescript data structures that would be useful during analysis.
- **dropkick-toolkit.ts** - convert DropkickReader-processed log data into a plot-able time series.
- **rr-geodesy.ts** - potentially useful, but not directly relevant to any of the analysis requirements mentioned so far.  Provides a function to accurately traverse anywhere on the surface of the earth.
- The [**geodesy**](https://www.npmjs.com/package/geodesy) npm package