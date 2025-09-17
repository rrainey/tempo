# Tempo Insights Web Application Architecture Plan

## Overview and Technology Stack  
Tempo Insights is a Next.js web application (TypeScript) running on a Raspberry Pi 5 (Ubuntu 24.04) designed for skydive jump log collection and analysis【11†L389-L397】. The app interfaces with **Tempo-BT** logging devices over Bluetooth and emphasizes minimal user intervention: a jumper simply powers on the device, makes a jump, and the system automatically discovers and uploads the jump log for later review【1†L2-L11】. Key technologies include the **Mantine** UI library for a dark-themed interface (using the provided color palette), **Recharts** for 2D charts, **D3** for interactive 3D skydive visualizations, **Prisma** as the ORM, and **Supabase (PostgreSQL)** as the database【11†L389-L397】. The app is deployed on-site at a drop zone to leverage local Bluetooth connectivity to devices【11†L398-L404】. 

**System Components:** The architecture consists of a Next.js server (serving pages and APIs) and two background Node.js processes for device communication and log analysis. A continuous **Bluetooth Discovery Service** runs as a separate process to scan for devices and handle log file uploads【9†L89-L98】【11†L478-L480】, while a scheduled **Log Analysis Worker** runs in parallel to process new jump logs and group formation skydives【9†L108-L116】【11†L442-L449】. The web application backend itself handles user interactions, API requests, and certain device management commands (like provisioning or blinking a device) by invoking Bluetooth functions on demand【11†L478-L480】. All components share a common data store (Supabase/Postgres) to communicate state and results. The entire UI is dark-mode only, styled via Mantine theming to use the specified color palette (e.g. `#002233` as primary background, `#ddff55` as emphasis)【11†L406-L415】【11†L482-L485】. 

**Bluetooth Interaction Tools** - this document cite several potential Bluetooth interface tools: notably, `smpmgr`, `mcumgr`, and `BlueZ`. As we began work and testing on the project, it became clear that a combination of `smpmgr` and the Linux `bluetoothctl` command line tool provided the interface capabilities required for the project.  Our implementation exclusively uses those.

**3D Graphics** -- `three.js` has been chosen over `D3.js` to allow for OpenGL rendering of graphics.  Replace any mention of `D3` with `three.js`.

## File and Folder Structure

### Actual Structure

This tree view depicts the current structure and progress towards MVP completion as of the end of Phase 11.  The target filename structure and documentation appears below as well.  **In some cases, there will be minor deviations in file naming.  Where these differences exist, the assigned name in this tree will take precedence for the project.**

```
tempo-insights/
    |
    ├── tsconfig.json
    ├── .editorconfig
    ├── .nvmrc
    ├── docs/
    │   ├── coordinate-frames.md
    │   ├── environment_setup.md
    │   ├── phase11-plan.md
    │   ├── sample-flight.txt
    │   ├── tempo-insights-requirements-outline.md
    │   ├── tempo_insights_architecture.md
    │   └── tempo_insights_mvp_plan.md
    ├── prisma/
    │   ├── schema.prisma
    │   └── seed.ts
    ├── scripts/
    │   └── test-jump-create.js
    ├── server/
    │   └── src/
    │       ├── TempoBTClient.ts
    │       └── TempoBTClient_examples.ts
    ├── smpmgr-extensions/
    │   ├── smpmgr-extension-commands.md
    │   └── plugins/
    │       ├── __init__.py
    │       └── tempo_group.py
    ├── src/
    │   ├── components/
    │   │   ├── AppLayout.tsx
    │   │   ├── AssignDeviceModal.tsx
    │   │   ├── AuthGuard.tsx
    │   │   ├── ChangePasswordModal.tsx
    │   │   ├── CreateGroupModal.tsx
    │   │   ├── EditNotesModal.tsx
    │   │   ├── InviteMemberModal.tsx
    │   │   ├── formation/
    │   │   │   ├── BaseInfoPanel.tsx
    │   │   │   ├── FormationReview.tsx
    │   │   │   ├── FormationViewer.tsx
    │   │   │   ├── JumperListPanel.tsx
    │   │   │   └── ViewControls.tsx
    │   │   └── home/
    │   │       ├── FormationJumpsPanel.tsx
    │   │       ├── JumpAltitudeChart.tsx
    │   │       ├── JumpDetailsPanel.tsx
    │   │       └── MyJumpsPanel.tsx
    │   ├── lib/
    │   │   ├── analysis/
    │   │   │   ├── dropkick-reader.ts
    │   │   │   ├── dropkick-tools.ts
    │   │   │   ├── event-detector.ts
    │   │   │   ├── kml-writer.ts
    │   │   │   ├── log-parser.ts
    │   │   │   ├── log-processor.ts
    │   │   │   └── rr-geodesy.ts
    │   │   ├── auth/
    │   │   │   ├── hash.test.ts
    │   │   │   ├── hash.ts
    │   │   │   ├── jwt.test.ts
    │   │   │   ├── jwt.ts
    │   │   │   ├── logout.ts
    │   │   │   ├── middleware.test.ts
    │   │   │   ├── middleware.ts
    │   │   │   └── verify-ssr.ts
    │   │   ├── bluetooth/
    │   │   │   └── bluetooth.service.ts
    │   │   ├── formation/
    │   │   │   ├── coordinates.ts
    │   │   │   ├── types.ts
    │   │   │   └── __tests__/
    │   │   │       └── coordinates.tests.ts
    │   │   └── utils/
    │   │       ├── slug.test.ts
    │   │       └── slug.ts
    │   ├── pages/
    │   │   ├── _app.tsx
    │   │   ├── _document.tsx
    │   │   ├── chart.tsx
    │   │   ├── devices.tsx
    │   │   ├── home.tsx
    │   │   ├── index.tsx
    │   │   ├── login.tsx
    │   │   ├── profile.tsx
    │   │   ├── register.tsx
    │   │   ├── test-altitude-chart.tsx
    │   │   ├── api/
    │   │   │   ├── hello.ts
    │   │   │   ├── admin/
    │   │   │   │   └── test.ts
    │   │   │   ├── auth/
    │   │   │   │   ├── login.ts
    │   │   │   │   ├── logout.ts
    │   │   │   │   ├── me.ts
    │   │   │   │   └── register.ts
    │   │   │   ├── devices/
    │   │   │   │   ├── list.ts
    │   │   │   │   └── [id]/
    │   │   │   │       ├── blink.ts
    │   │   │   │       └── unprovision.ts
    │   │   │   ├── dropzones/
    │   │   │   │   ├── [id].ts
    │   │   │   │   └── index.ts
    │   │   │   ├── formations/
    │   │   │   │   ├── [id].ts
    │   │   │   │   └── mine.ts
    │   │   │   ├── groups/
    │   │   │   │   ├── [slug].ts
    │   │   │   │   ├── index.ts
    │   │   │   │   └── [slug]/
    │   │   │   │       ├── invite.ts
    │   │   │   │       └── join.ts
    │   │   │   ├── internal/
    │   │   │   │   └── jumps/
    │   │   │   │       └── create.ts
    │   │   │   ├── invitations/
    │   │   │   │   └── [code]/
    │   │   │   │       ├── accept.ts
    │   │   │   │       └── decline.ts
    │   │   │   ├── jumps/
    │   │   │   │   ├── [id].ts
    │   │   │   │   ├── mine.ts
    │   │   │   │   └── [id]/
    │   │   │   │       ├── notes.ts
    │   │   │   │       └── visibility.ts
    │   │   │   └── users/
    │   │   │       ├── invitations.ts
    │   │   │       ├── me.ts
    │   │   │       └── password.tsx
    │   │   ├── dropzones/
    │   │   │   ├── [id].tsx
    │   │   │   └── index.tsx
    │   │   ├── groups/
    │   │   │   ├── [slug].tsx
    │   │   │   └── index.tsx
    │   │   ├── jumps/
    │   │   │   └── [id].tsx
    │   │   ├── review/
    │   │   │   └── fs/
    │   │   │       ├── [id].tsx
    │   │   │       └── index.tsx
    │   │   └── users/
    │   │       └── [slug].tsx
    │   ├── scripts/
    │   │   ├── test-bluetooth-scan.ts
    │   │   ├── test-event-detector.ts
    │   │   ├── test-file-upload.ts
    │   │   ├── test-idempotency.ts
    │   │   ├── test-lastseen-updates.ts
    │   │   ├── test-log-processor.ts
    │   │   ├── test-parser.ts
    │   │   ├── test-smpmgr-integration.ts
    │   │   └── test-visibility.ts
    │   ├── styles/
    │   │   ├── globals.css
    │   │   ├── Home.module.css
    │   │   └── theme.ts
    │   └── workers/
    │       ├── bluetooth-scanner.test.ts
    │       ├── bluetooth-scanner.ts
    │       └── log-processor.ts
    ├── supabase/
    │   └── config.toml
    └── .github/
        └── workflows/
            └── ci.yml
```

The project follows a structured Next.js convention, organized by feature and responsibility for clarity and scalability. Below is an outline of the key folders and files, along with their purposes:

pages/ – Next.js page components defining the routing structure of the app. Each file or subfolder corresponds to a route:

pages/index.tsx – The main entry (may simply redirect to /login or /home based on auth status).

pages/login.tsx – Login page with a form for email and password authentication.

pages/register.tsx – User registration page for creating a new account.

pages/accept-invitation/[invitationId].tsx – Account claiming page for proxy users. This page is accessed via a one-time invite link and allows a Proxy User (created during device lending) to set their email/password and convert into a full account.

pages/profile.tsx – User profile settings page. Allows editing of the user’s name, profile photo, password, etc. (accessible when logged in).

pages/home.tsx – The user’s main dashboard/home page after login. This page includes a three-column layout:

### Left Navigation – Links to the user’s Groups, Devices, Settings (profile), and Logout.

### Center Panel – Jump review area that shows details or visualizations for the selected jump (e.g. charts of a jump’s altitude or fall rate). If a jump is part of a formation skydive, it may include a preview or link to the formation visualization.

### Right Sidebar – Summary panels, e.g. “Formation Jumps” (list of recent formation skydive events involving the user), “My Jumps” (list of the user’s recent solo jumps), and a Summary of the currently selected jump (date/time, exit altitude, freefall time, average fall rate, etc.).

pages/devices.tsx – Device management page (visible to Administrators only) for listing and administering all known devices. This page displays each device with its status and offers management actions via a kebab menu. (This route is implied by the requirements for “Device Management” functionality).

pages/users/[userSlug].tsx – Public profile page for a user, identified by their slug (unique username URL). This page shows a user’s public details and shared jump statistics. It also allows sending connection requests (friend requests) if not already connected.

pages/groups/[groupSlug].tsx – Group page for a skydiving group/community. Shows group info, members list, and recent jumps visible to the group. If the viewer is an admin of the group, they can invite new members or manage settings. Non-members can request to join if the group is public.

pages/review/fs/[fsId].tsx – Formation Skydive review page for a specific formation jump event. This page provides a focused view of the multi-jumper skydive: an interactive 3D visualization (using D3) of the skydive “movie” and sidebar panels with details. The UI includes a list of participating jumpers and a “Base Info” panel showing the base jumper’s fall rate (actual vs. calibrated) and other stats.

Purpose: The pages/ directory defines all primary user-facing endpoints and page layouts. Each page component uses Mantine UI components for consistent styling, and most pages fetch required data via Next.js data fetching methods or by calling internal APIs (see pages/api/). Pages enforce access control (e.g. redirect to login if not authenticated, or restrict admin pages).

pages/api/ – Next.js API route handlers for backend functionality (executed on the server side). This folder contains server endpoints that the front-end can call (via fetch or Axios) for actions and data. Key API routes might include:

api/auth/login.ts – Verifies credentials and creates a session token (e.g., JWT or secure cookie) for the user upon login. On success, the token (valid ~30 days) is returned or set as HttpOnly cookie.

api/auth/register.ts – Creates a new user (hashing the password securely) and possibly sends verification (for now, likely just creates the account).

api/devices/[deviceId]/assign.ts – Assigns or provisions a device to a user (Administrator-only). It uses the Bluetooth service/library to connect to the device and set its new User ID and unique device ID if needed. Also updates the database device record (owner user, state = “Assigned”, etc.).

api/devices/[deviceId]/unprovision.ts – Unassigns a device (resetting it to unprovisioned state). Uses the Bluetooth interface to reset the device’s stored ID and clears assignment in the DB.

api/devices/[deviceId]/blink.ts – Triggers the device’s blink command to flash its LED for identification (calls the Bluetooth shell command via our service).

api/devices/list.ts – Returns the list of devices and their status (for the Devices page). This reads from the database (which is continually updated by the Bluetooth service with online/offline status).

api/logs/[logId] – Protected endpoint to fetch a jump log’s detailed data (or allow downloading the raw log file). Could also handle the “download all my data as zip” request by zipping all of a user’s logs.

api/groups/[groupId]/invite.ts – Creates an invitation for a user to join a private group (for group admins).

api/users/[userId]/connections.ts – Handles sending or responding to connection requests between users (friend requests).

api/lending/create-proxy.ts – Creates a Proxy User entry and a one-time claim invitation (used when lending to a brand new person).

api/lending/lend.ts – Endpoint to record a device lending transaction. It updates the device’s record with the lent-to user and lending period (one jump or until reclaim). If lending to a new person, it calls create-proxy first; if lending to an existing user, it just updates the fields. It may also call the device provisioning routine to update the device’s on-device assigned user ID to the borrower so that the upcoming logs attribute correctly.

(Many of the above could also be grouped logically, e.g. a single devices.ts file with multiple actions, or separated as shown. The exact routing can vary, but the above outlines key backend interactions.)

Purpose: The api/ routes implement the server-side logic for authentication, device control, data queries, and so forth. They use Prisma to interact with the database and may call out to local system utilities or services (for Bluetooth actions). By structuring the backend as Next.js API routes, we keep a unified codebase and can easily secure these endpoints (checking user roles, etc.).

components/ – Reusable UI components and widgets, organized by domain or feature. This directory holds the building blocks of pages, often split into subfolders:

components/layout/ – Shared layout components, e.g. a <NavBar> for the left navigation menu (showing “Groups”, “Devices”, etc.), a <Header> for top bar if needed, or <ProtectedRoute> wrapper to enforce auth on pages.

components/devices/ – UI pieces for the Devices management page. For example, <DeviceList> to list all devices with their status, <DeviceItem> for a single device entry (showing name, online/offline indicator, and actions menu), and modal components like <AssignDeviceModal> or <ConfirmUnprovisionModal> for admin actions.

components/jumps/ – Components related to jump log display. For instance, <JumpSummaryCard> for showing basic info about a jump (used in the “My Jumps” list or summary panel), or <JumpChart> which might use Recharts to plot altitude vs time or fall rate graphs for a single jump.

components/formation/ – The 3D visualization and controls for formation skydive review. This might include a <FormationViz> component that encapsulates the D3 logic to render the “movie” of the skydive in a canvas or SVG. It would handle the coordinate transformation of GPS and altitude data into the formation frame (with +X axis along aircraft track, +Z up) and provide controls for play/pause, viewpoint toggling (god’s-eye vs side view) and rotation. Additional components here: <JumperLegend> to show each jumper’s current speed (mph) and altitude, and highlight the base jumper’s data, and <TimelineControls> for the playback controls.

components/groups/ – Components for group pages, e.g. <GroupMemberList>, <InviteMemberForm>, etc.

components/lending/ – Components to support the lending workflow. For example, <LendDeviceForm> that appears on the “Lend My Device” page where a user selects a device, chooses an existing user or enters a new name (triggering proxy creation) and sets the lending duration. Also, <ProxyUserList> might list proxy users a user has created (for account claiming).

components/common/ – Any generic components or utilities, like form input components, buttons, or confirmation dialogs that are reused across features.

Purpose: This folder keeps the UI modular. By separating components by feature, we promote reuse and clarity. Each component is typically a functional React component with Mantine styling. State within these is generally local (e.g. form state) or passed in as props; larger state (like global user info or theme) comes from Context or props.

services/ (or lib/) – Application logic and integration layers that are not React components. These TypeScript modules handle business logic, external interactions (Bluetooth, etc.), and database queries in a structured way. Key service modules include:

services/authService.ts – Handles authentication logic (e.g. verifying passwords, hashing new passwords, issuing JWT or session tokens). It would interface with Prisma to find user by email and compare hash.

services/userService.ts – Functions for user management: creating users (setting initial roles), retrieving user profiles, handling connection requests, and enforcing that Proxy Users cannot perform certain actions (e.g. cannot create groups or have admin role).

services/deviceService.ts – Core logic for device provisioning and control. This service uses the Bluetooth interface to communicate with devices. For example, deviceService.assignToUser(deviceId, userId, nextJumpNo) would call the underlying Bluetooth library to perform the provisioning operation (writing the new user’s UUID and setting the next jump number on the device). It also updates the Devices table in the database accordingly (marking the device as assigned/provisioned to that user, updating its state from “Provisioned” to “Assigned”). Other functions: blinkDevice(id) to call the blink command, unprovisionDevice(id) to reset a device (both in DB and on the device itself).

services/bluetooth.ts – Bluetooth Interface Library (the integration to `smpmgr`). This is a critical module (possibly provided externally) that abstracts low-level BLE operations. It might be structured as a class (e.g. TempoBluetoothManager) with methods to scan for devices, connect and run commands, and transfer files. Under the hood, it likely invokes the mcumgr CLI tool via shell calls (for example, using Node’s child_process to run mcumgr commands to list files or read/write to the device’s shell). This library would handle retries and error handling for file transfers, given the potential instability of BLE. In an ideal scenario, it might interface directly with BlueZ through D-Bus for efficiency, but using mcumgr is a reliable fallback. Both the Device Service (for on-demand actions) and the background Bluetooth service (for continuous scanning) use this library.

services/logService.ts – Manages jump log records. It can include logic to save a received log blob into the database (via Prisma or Supabase client) and compute any immediate values. For example, when a new log is uploaded, this service might calculate the next jump number for that user and fill it in the record, or ensure the SHA-256 hash of the file is stored. It might also expose methods to retrieve logs (with filtering by user or group visibility rules).

services/analysisService.ts – Functions used by the analysis worker (possibly shared or separate in the worker code). This would include the implementation of the Initial Analysis algorithms: reading raw log data and extracting key event timestamps (exit, parachute activation, deployment, landing), estimating exit altitude and position, and calculating fall rates. It also includes logic for Formation Skydive detection – comparing a log’s start timestamp against others to determine if it falls within a 120-second window of another jump, thereby grouping those logs under a formation event. Additionally, this module would handle fall rate normalization calculations: given a series of altitude readings, it adjusts the fall rate for air density at each altitude to produce a normalized fall rate curve. This is used for the performance assessment feature and for showing calibrated fall rates (e.g., for the base jumper in a formation).

services/groupService.ts – Handles creation of groups, joining logic, and invitations. It enforces that a Proxy User cannot create groups. It also ensures group membership rules: e.g. adding a user to a group (with isAdmin flag if they created it), listing public groups, processing join requests and invitations.

services/lendingService.ts – Coordinates the device lending process. For example, lendDevice(ownerUserId, deviceId, targetUserId or newUserInfo, duration) will create a Proxy User if needed, update the device’s record to mark it lent (store lentTo user and lending type), and possibly trigger the on-device user assignment update via deviceService (so that the device knows to log under the new user’s ID). It also would handle auto-return logic: if the lending period is “one jump”, this service (or the Bluetooth log uploader) will detect when one new log has been uploaded and then automatically revert the device assignment back to the owner. The reversion can be done by updating the device’s DB record (clearing lentTo and setting state back to “Assigned” to owner) and by writing the owner’s user ID back to the device’s config. If the lending period is “until reclaimed”, the device stays with the lent user until an explicit reclaim action (which would be another API call, e.g. api/lending/reclaim, that uses this service to revert the assignment).

services/notificationService.ts (optional) – If the app needs to send emails (for invitations or notifications), this would handle sending a “Claim Account” email or generating a QR code image to display for the claim URL. However, the requirements suggest the claim link is shown as a QR code in-app rather than emailed, so this might be a minor utility.

Purpose: The services layer encapsulates non-UI logic and integrations, making it easier to maintain and test business rules. By separating these from React components, we ensure the components focus only on presentation and user interaction, while services handle data fetching, device I/O, and computation.

workers/ – Standalone Node.js scripts or modules for background processes that run alongside the Next.js server. We have two main workers:

workers/bluetoothScanner.ts – Implements the Log Discovery and Upload Service that runs continuously on the server. This script uses the services/bluetooth library to interface with the Bluetooth adapter. It performs a BLE scan (using BlueZ or mcumgr) for devices advertising as “Tempo-BT-xxxx”. It keeps an in-memory table (or uses the DB) of detected devices and their last-seen timestamps. For any known device that is seen after being absent for a period (> DISCOVERY_WINDOW, e.g. 300 seconds), it assumes the device has new data (e.g. the user just completed a jump and came within range). The scanner then connects to that device and queries its file system (via mcumgr fs commands or a custom GATT service) to list log files. By comparing with what’s already uploaded (the service can store the list of file names or hashes it has seen), it identifies any new log files. Each new file is then transferred from the device to the server (using the mcumgr file transfer command) and saved to the database as a new Jump Log record. The Jump Log record includes metadata from the device: the log’s timestamp (estimated exit time), the device’s user ID (to associate the log with the correct user in the DB), and a computed SHA-256 of the file for integrity. After a successful upload, the service may optionally command the device to mark that file as uploaded or simply rely on stored filenames to not re-upload duplicates. This service also updates each device’s online status in the database: when a device is detected via scanning, mark it “Online” (and record the timestamp); if a device hasn’t been seen for > DISCOVERY_WINDOW seconds, mark it “Offline”. The online status (and potentially a last-seen timestamp) is used by the UI to show device connectivity. The scanner runs in an endless loop or event-based on Bluetooth callbacks. It should handle multiple devices sequentially – e.g., if several devices come online at once, it queues the file transfers to not overwhelm the bandwidth. This process runs independently of the web server, likely launched via a system service or alongside the Next.js app using a process manager, ensuring it starts at boot.

workers/logProcessor.ts – Implements the Preliminary Jump Analysis background job. This script runs periodically (for example, using setInterval or a simple sleep loop waking every 30 seconds) to check for any new jump logs that need analysis. It queries the database for all Jump Log records where initialAnalysisTimestamp is NULL (meaning not yet processed). Those records are then processed one by one (serially) – avoiding parallel processing simplifies load on a small device and ensures formation grouping logic can easily find existing logs. For each log:

It loads the raw log data (the binary/blob) and runs analysis to extract key events and metrics. Using the logic in analysisService, it determines:

Exit time offset – when freefall likely began (e.g., first moment vertical speed exceeds 2000 fpm).

Parachute activation – when the parachute was deployed (e.g., first significant deceleration >0.25g).

Parachute deployment complete – when descent rate fell below 2000 fpm after freefall, marking canopy flight start.

Landing time – when descent rate drops below ~100 fpm for a sustained period, indicating landing.

Exit timestamp (UTC) – the absolute timestamp of exit. This is computed by taking the log’s start timestamp (when device started recording on the plane) plus the exit offset in seconds. The log’s start time might come from device’s internal clock or GPS time at start.

Exit position (lat/lon) – the GPS coordinates at the exit moment (if the device logs GPS and has a 3D GPS fix at exit). This could be null if not available.

These values are then saved back to the Jump Log record in the database (fields for each metric). Additionally, if any anomalies or processing notes occurred, the initialAnalysisMessage field can be filled for reference.

After populating the log’s analysis fields, the worker then performs Formation Skydive grouping logic. It compares the current log’s start time or exit time with other logs in the system. If it finds any other jump log(s) whose start times are within ±120 seconds of this log’s start, it considers them part of the same skydive formation. The worker will then either create a new Formation record or update an existing one:

If neither log was previously in a formation, a new FormationSkydive entry is created in the DB with a new UUID. The formation record will contain a map or list of the involved Jump Log IDs (participants). It marks one of them as the “base” jumper by default (perhaps the first log processed or arbitrarily). (Participants can later change which jumper is base via the UI, since base designation is editable by any participant.)

If one of the logs is already associated with an existing formation (e.g., earlier a formation was created when a previous log was processed), the worker will add this new log to that existing formation’s participant list.

For each link between a formation and a log, the isVisibleToOthers flag is set initially true. (If a user has marked their log as private to themselves by unchecking “Visible to Connections”, then in the formation context that participant’s data would be hidden to others, but the formation record itself still exists with other participants’ data visible. The UI will respect each participant’s visibility setting.)

The formation’s date/time is recorded (likely as the exit date/time, maybe using the first two logs’ times to determine it).

Once the log is analyzed and (if applicable) formation grouping updated, the worker sets the log’s initialAnalysisTimestamp to now (marking it as processed). It then moves to the next queued log. After processing all pending logs, the worker sleeps until the next cycle. Because the queue is derived from the database query each time, the system is robust to restarts and doesn’t require an external queue manager (no Redis needed).

The log processor runs as a separate Node process (or thread). This isolation ensures that heavy computation (parsing large 8MB log files, performing numeric analysis) does not block the Next.js web server. On a Raspberry Pi, this separation helps utilize multiple cores and keep the UI responsive. The web application and the worker communicate only through the database: new logs inserted by the Bluetooth service signal work for the processor (via the NULL initialAnalysisTimestamp), and the results written by the processor are later read by the UI when needed.

Purpose: The workers processes implement continuous and periodic background tasks that are essential to the application’s real-time data collection and analysis. By running them in parallel to the main web app, we adhere to the real-time requirements without requiring the user to manually trigger these actions, and we keep the architecture decoupled (the web UI doesn’t need to stay open for these tasks to run).

prisma/ – Prisma ORM files and database schema:

prisma/schema.prisma – The Prisma schema defining the database structure (tables and relations). It will include models for Users, JumpLogs, Devices, Groups, GroupMemberships, Formations, etc. For example, the User model with fields: id (UUID), fullName, email, passwordHash, isProxy (flag), proxyCreatorId (nullable foreign key if this is a proxy user created by someone), createdAt, lastActiveAt, and a unique slug for profile URL. The UserRole or roles model linking users to roles (like “Administrator”). The Device model with fields as described: id (UUID), ownerId (User who owns it), lentToId (User who currently has it, if any), state (enum: Unprovisioned/Provisioned/Assigned), bluetoothId & name, deviceUID (hardware identifier if any), onlineStatus (bool or lastSeen timestamp), lendingPolicy (enum or flags for one-jump or until reclaim). The JumpLog model with fields: id (UUID), userId (owner), deviceId, jumpNumber (the user’s jump count), startTimestamp (UTC of log start), exitTimestamp (UTC estimated exit time), exitLat, exitLon, exitAlt, freefallTime, deploymentTime (timestamps or offsets), landingTime, rawLog (binary blob of log file), logHash (SHA-256), notes (text), visibleToConnections (bool), initialAnalysisTimestamp, initialAnalysisMessage. The FormationSkydive model with fields: id (UUID), maybe formationTimestamp or date, baseLogId (which jump log is base), and a relation to FormationParticipants or a JSON map of participant logs with their visibility flags. Many-to-many relations like User-Group membership (with an isAdmin flag), or User-User connections can be represented as join tables as well. This schema is the single source of truth for data structure.

prisma/seed.ts – A seed script to initialize the database with an initial Administrator user (since one admin must exist to manage devices). This creates a user with Admin role on first run.

The Prisma client is generated and used in the app (for example, in services/* and api/* routes) to query and mutate data. Supabase is essentially used as a hosted Postgres; Prisma will connect via a connection string. We store jump log blobs directly in the DB (likely as a bytea or large text column) as allowed, avoiding the complexity of external object storage in this phase.

### Other Config/Utility Files:

next.config.js – Next.js configuration (if needed for custom webpack or to enable experimental features).

.env – Environment variables for sensitive config (e.g., database URL for Supabase, secrets for JWT, etc.). This will also hold configuration constants like DISCOVERY_WINDOW default (e.g. 300 seconds) if we choose to make it configurable without code changes.

utils/constants.ts – A TypeScript module exporting various constant values used across the app. This includes thresholds and tunable parameters such as DISCOVERY_WINDOW (BLE scan interval/window), fall rate baseline thresholds (default min 110 mph, max 130 mph for average jumper), or other fixed options. Storing them in one place makes it easy to adjust and document these values. In the future, these could be loaded from the database (for instance, an “AppSettings” table) to allow runtime adjustment via an admin UI, but for now a constants file or env is sufficient.

utils/formatters.ts – Utility functions for formatting data for display (e.g., converting timestamps to local dropzone time zone, formatting altitude or speed with units).

utils/validators.ts – Functions to validate input (like password strength, proper email format, etc.) used on the front-end and/or back-end.

theme.ts – Mantine theme extension: defines the dark theme palette using the provided colors. For example, setting Mantine’s primaryColor or theme overrides to use #ddff55 or #855bf0 for accent elements, and setting global styles for background to #002233 etc. This file exports a Mantine ThemeOverride object used in the Mantine provider.

pages/_app.tsx – The custom Next.js App component. This is where we integrate providers that wrap the entire app. We include <MantineProvider> here, configuring it for dark mode and injecting our custom theme colors. We also enable Mantine’s default fonts (it will use platform defaults as recommended). Additionally, we can set up a Context Provider for authentication state here (e.g., AuthProvider) to make the logged-in user info and roles available throughout the component tree.

pages/_document.tsx – Custom Document for Next.js, used to augment the HTML document. If Mantine requires server-side style injection, we would configure it here (Mantine has SSR support to avoid FOUC). This can also preload any static assets or set lang attributes as needed.

This structured file layout ensures clear separation of concerns: pages define the views and fetch data, components encapsulate UI elements, services handle logic/IO, and workers handle background tasks. All code is in TypeScript for type safety, and the folder names are meaningful (e.g., “components”, “services” instead of generic “utils”)
medium.com
 for clarity.

## State Management and Communication

State in Tempo Insights is managed at three levels: local UI state, shared client state via context, and persistent state in the database. Below is how each is handled and how different parts of the system communicate:

Local Component State: Individual React components use local state (via hooks like useState) for ephemeral UI state – form inputs, open/closed states of dialogs, current playback time in a visualization, etc. For example, the FormationViz component might keep state for the current playback timestamp or current camera angle. This state is not needed outside the component and resets when the component unmounts.

Global Client State (Context): Some state is shared across the app and needs to be accessible by many components. We utilize React Context for these cases:

Authentication Context: After a user logs in, we store the user’s session info (e.g., user ID, name, roles, and session token) in context (or use Next.js built-in mechanisms if using something like NextAuth, but here likely a custom context). This avoids prop-drilling the user info into every page and allows any component to check authContext.currentUser or similar to get the logged-in user. The context is provided in _app.tsx and populated on login or page load (possibly via an getServerSideProps that validates the session token and provides user info).

Theme Context: Mantine’s provider largely handles theming, but we might also keep a context if needed for toggling theme. However, since the app is dark-mode only at MVP, we likely hardcode dark theme. In future, a context could toggle light/dark.

Device/Status Context (optional): While not strictly necessary, we might have a context or state management for real-time device status or notifications. For example, a context that holds a list of devices with their online/offline status, updated periodically. However, since device status can be fetched on demand and via polling (see below), we might avoid duplicating it in context and instead rely on React query or simple polling state.

Persistent State (Database): Almost all long-term state and data reside in the Supabase (Postgres) database via Prisma. This includes user profiles, device info, jump logs, etc. The database is the single source of truth for current device assignments, jump log contents, analysis results, group memberships, and so on. When the front-end needs to know something (e.g., “what jumps has this user done?” or “is device X online?”), it requests data from the backend, which in turn queries the database (or sometimes the in-memory data updated by a service). By storing state server-side, we ensure data consistency and security (clients only see what they are authorized to see, via the controlled API responses).

Client-Server Communication: The front-end uses Next.js API routes (or server-rendered data) to get updates. For example, the Devices page will fetch the latest device list and statuses via GET /api/devices/list when it loads, and then refresh that data every 15 seconds in the background to reflect any changes (like a device going offline or a new device appearing). This polling mechanism is sufficient given the relatively infrequent changes and avoids the complexity of maintaining WebSocket connections, which are explicitly not required. We might use a hook with setInterval or a library like SWR/React Query for polling. Similarly, on the home page or any place showing jump data, we can poll or use server-sent events if needed, but likely polling is enough for updating a new jump that was just uploaded by the background service (e.g., query every minute for new jumps if user is viewing a list).

Inter-service Communication: The web application (Next.js) and the background services (Bluetooth scanner and log processor) do not directly call each other’s functions; instead, they communicate via the database and, in some cases, via the file system or OS signals:

The Bluetooth scanner service writes to the database when it discovers devices and uploads logs. For instance, when it transfers a new log file, it uses Prisma (or Supabase client) to insert a new JumpLog record with all relevant fields. It also updates the device’s lastSeen timestamp and online status in the Devices table each time it detects a device. The web app’s device API (and thus the UI) reads those fields to display status. If the scanner detects a new unprovisioned device (Bluetooth name "Tempo-BT-unprovisioned"), it will insert a record in Devices table with state=Unprovisioned so that an admin viewing the Devices page will see it available for provisioning. This loose coupling means the front-end always reads from the DB as the source of truth, and the scanner only needs DB write permissions.

The Log processing worker similarly uses the database as its communication medium. It reads unprocessed logs from DB, writes analysis results and formation group records back. The formation group linking is in the DB, so when a user opens their home page, the app can query: “give me all FormationSkydive records where this user is a participant” to list their formation jumps. Likewise, after analysis, a jump log now has fields like exitTime, freefallDuration filled in; these can be immediately shown on the UI (e.g., in the jump summary panel) once the UI fetches that log data.

There is minimal direct interaction needed between the web server and background processes. One small area of coordination: Device provisioning commands (triggered by admin actions) versus the continuous scanner. For instance, if an admin clicks “Assign to User” on a device at the same time the scanner might be attempting to fetch logs, we need to ensure the Bluetooth operations don’t collide. This can be handled by the Bluetooth library instance: it might maintain a lock per device so that only one operation (either a user-triggered command or an automatic log fetch) happens at a time. The services could share a common queue or locking mechanism in memory or via a file lock. Given that the provisioning commands are part of the web backend (perhaps executed via the deviceService), one approach is to have the scanner process and web process not run Bluetooth actions concurrently. For simplicity, one could pause scanning briefly when a manual operation is requested. Alternatively, if using BlueZ natively, multiple connections might be handled gracefully by BlueZ as long as they target different GATT operations. This is an implementation detail; architecturally it’s noted that the Bluetooth subsystem must coordinate access between automatic and manual tasks.

State and UI Updates: For dynamic aspects like device status, the UI relies on periodic refresh. Any page showing device connectivity (e.g. Devices management page) will trigger a refresh action (setInterval to call the API) to update the list at least every 15 seconds. Similarly, the home page might periodically refresh the list of recent jumps or formation jumps to catch any new logs uploaded in near real-time. Since logs uploads happen automatically after a jump, a user who just landed could expect their jump to appear in the UI within seconds of the device being in range. A short polling interval (30s or 60s on the jumps list) could catch this, or the user can manually refresh. WebSockets are not required, so this simple polling meets the real-time requirement without additional infrastructure.

Session Management: Authentication uses a session token mechanism. When the user logs in with email and password (which are verified against the hashed password in the DB), the server creates a session token (for example, a JWT signed with a secret or a random token stored in a Session table). This token is sent to the client as a secure, HttpOnly cookie. The cookie expiry is set to 30 days, so the user stays logged in for up to a month unless they log out. On each request, the server can validate the token and identify the user. We do not rely on Supabase’s built-in auth in this architecture; instead, we implement custom auth to allow our own user and role management. All sensitive data transmissions (login, registration) occur over HTTPS, and passwords are stored as secure hashes (bcrypt or similar) with salts. There are no special encryption requirements beyond standard web security (HTTPS, secure cookies) in this setup. The Bluetooth communication to devices in this version is also not encrypted/paired (devices are discovered and connected openly), but this is acceptable given the local network scope and the MVP requirements.

In summary, state is largely centralized in the database for consistency, while React context and component state manage the UI-specific and session-specific aspects. The app’s moving parts (front-end, API, Bluetooth service, analysis worker) stay in sync by reading/writing the shared database and by the front-end polling for updates as needed.

## User Management and Security (Registration, Roles, and Profiles)

User accounts are the cornerstone of application security and personalization. The architecture supports robust user management as described in the requirements:

Registration & Authentication: New users register by providing a full name, email, and password. The password is immediately hashed (using a strong algorithm like bcrypt) and only the hash is stored. User creation is handled by the register API route and userService, which also generates a unique URL-friendly slug for the user (based on their name). After registration, the user can log in via the login page, which calls the login API. Successful login returns a session token (cookie) as described, establishing the user’s session. There is no third-party auth in MVP, and no email verification flow mentioned, so accounts are active immediately. The initial deployment will include a default Administrator account (created via seeding) to bootstrap the system.

Roles and Permissions: The system currently defines one special role: Administrator. The first user (seeded) is an Admin by default. The Roles are stored in a separate relation (e.g., a UserRoles table linking userId and role name), allowing for future expansion. Role checks are enforced in the API routes and front-end:

Only Admins see the “Device Management” menu and can access /devices page.

Only Admins can perform device provisioning actions (assign, unprovision, initialize).

Admins can see all devices in the system, whereas a normal user might only see devices they own or that are lent to them.

Group creation is open to any logged-in user (they become admin of the group they create), but Proxy Users (see below) cannot create groups or be assigned roles.

These checks are implemented in both UI (e.g., conditional rendering of admin menus) and API (verifying user.role == Admin on protected endpoints).

User Profiles and Connections: Each user has a profile (editable on /profile). They can update their name, upload a profile picture (stored as a blob in the DB, with MIME type), and change password. Users are identified publicly by their name and slug. The app allows users to find each other and connect:

There is a user search by name feature (likely on the “Connections” or group invite dialogs).

A user can send a Connection Request to another user to become “connections” (akin to friends/followers). Pending connection requests (and group invites) are shown in a “Pending Actions” list in the UI for the user to Accept or Decline.

Being “connected” (or being in a common group) influences data visibility: by default, when a new jump log is uploaded for a user, it is marked “Visible to Connections”. This means that other users who are connected (friends) or who share a group with this user will be able to see that jump in their views. If the user desires privacy for a particular jump, they can mark it as not visible, and then only they (and admins) can see it. This privacy flag is stored in each JumpLog record.

Public facing pages (/users/[slug]) can show some summary of a user’s jumps that are visible to the viewer (if viewer is a connection or groupmate and the jumps are marked visible). If the viewer is not allowed, it may show limited info or prompt to connect.

The architecture uses the database to enforce these rules, e.g., queries for jumps on another user will filter out those not visible or not permitted for the requesting user. We also incorporate these rules into UI logic (not showing certain data if not allowed) for defense-in-depth.

Groups: Groups provide a way to organize users (e.g., a dropzone group, a skydiving team, etc.). A group has a name and description and a unique slug. Users can join multiple groups. Key points:

Creation: Any full user can create a group from the UI (e.g., a “Create Group” button). They become the group’s administrator by default. GroupService will generate a slug and save the group with the creator listed as admin member.

Privacy: Groups can be Public or Private. Public groups are listed in a Groups catalog page, and any user can join them freely. Private groups do not show up publicly; membership is by invitation only.

Joining & Invites: To join a public group, a user simply clicks “Join” and the membership is added. For a private group, a group admin must send an invite to a user. This might be implemented by an API call to create a GroupInvite entry (or just directly adding membership in a pending state). The invited user will see a pending group invitation in their “Pending Actions” and can accept or decline. Accepting adds them as a member; declining discards the invite.

Group Page and Data: On a group’s page (/groups/[slug]), members can see all other members and possibly a feed of recent jumps by members that are shared to the group. By design, if two users are in the same group, they gain the ability to see each other’s jump logs that are marked visible to connections. The group page could list recent formation skydives involving multiple group members, or statistics aggregated from member data. Group admins might also manage membership from this page (invite/remove members).

Administration: A group can have multiple admins. The creator is an admin; they can promote other members to admin status (the GroupMembership model has an isAdmin flag). Admins can remove members or disband the group. If an admin tries to leave or delete a group and they are the only admin, the UI should prompt them to assign a new admin (or else the group might become ownerless). These edge cases are handled in the groupService logic.

Proxy Users and Account Claiming: A Proxy User is a special user account created as a placeholder when lending a device to someone who doesn’t have an account. In the database, a Proxy User is simply a User with an isProxy flag true and no login credentials initially (email may be blank or a dummy, and no password set). The proxy is linked to the real user who created it via a proxyCreatorId (the lender). Proxy Users behave like regular users in that they can accumulate jump logs (the logs from the lent device will be associated with the proxy’s userId) and they appear in connections or group contexts (the lender can see them and their data, and potentially others if connected).

The system prevents Proxy Users from logging in (since they have no password) and from certain actions (they cannot create groups or be given roles).

When a device is lent to a new person, the lending workflow (via lendDevice service) will create a Proxy User with the provided name. The lender can then immediately lend the device to this proxy (set device.lentTo = proxy’s id). From the UI perspective, the lender might not even realize a separate “proxy account” was created; they just see the name of the person.

Later, if that person wants to join the platform fully, the lender can assist them in claiming the account. The “Help Someone Claim Account” feature lets the lender select one of their proxy users and generate a Claim Invitation. This creates a secure token or record (UserInvitation model) with an expiry (e.g., 30 days) and ties it to the proxy user’s ID. It produces a unique URL (including that token or an invitation ID) for account claim. The UI shows this as a QR code for convenience.

When the proxy user scans or clicks the claim URL, they reach the accept-invitation/[id] page. This page verifies the token, and if valid, allows the person to set up their email and password (essentially completing the registration for that Proxy account). Upon submission, the backend will update the User record: set their email, set a password hash, and flip isProxy to false (making them a normal user). Now they can log in with that account going forward, and they retain all their jump data that was collected while they were a proxy (since it’s the same user record, just now with credentials). The invitation token is one-time use and expires after use or after 30 days.

Security for this process relies on the uniqueness of the invitation URL and possibly a short expiry to prevent misuse. Only someone with the link (or QR) can claim, and once claimed, the link is invalid.

Data Privacy and User Data Control: By default, all jump logs a user creates are private to them except where shared via groups or connections with the “visibleToConnections” flag. We have covered how connection-based visibility works. Additional user controls:

A user (or proxy’s creator) can delete data: The requirements note that users should be able to permanently delete their jump logs or even delete a proxy user they created, and if they are the sole admin of a group, to delete that group (with reassignment option for admin). Architecturally, this means providing API endpoints to perform deletions: e.g., DELETE /api/logs/[id] to delete a jump (if it belongs to the user and perhaps also remove it from any formation group), DELETE /api/users/[proxyId] to remove a proxy user (only allowed if you are the creator and the proxy has no data needed, or perhaps allowed regardless with data reassigned or removed). Group deletion will likely check membership and either transfer admin or prevent deletion if others are still in the group, as noted.

Data export: A user can request to download all data associated with their account as a ZIP file. This could be initiated via a button on profile or settings. The backend would gather all their jump logs (maybe in JSON or CSV, plus their raw log files), profile info, group memberships, etc., and bundle it. This might be handled by a utility function in the logService or a dedicated exportService. Since this can be heavy, it might be generated on-the-fly and streamed, or prepared asynchronously (but for MVP, on-demand generation is fine given moderate data sizes).

Security is maintained by ensuring every API route checks the user’s session and appropriate authorization. Also, any time a user tries to access data not theirs (like viewing another’s jump), the server verifies the relationship (connection or group) and the visibility flag before allowing it. Sensitive operations (device assignment, data deletion, group admin actions) double-check roles and ownership. With password hashing and HTTPS in place, and no additional backdoors (the Bluetooth service runs locally and trusts the device outputs), the architecture meets the outlined security expectations for this stage.

## Device Provisioning and Management via Bluetooth

Device management is a central feature, especially given the need to interface with hardware. Tempo Insights supports full lifecycle management of Tempo-BT devices: from factory initialization to user assignment, including lending and maintenance actions. The architecture divides this into two parts: background device discovery and on-demand provisioning commands.

Device States and Database: Each device is tracked in the database with its state and associations:

- Unprovisioned: Just flashed, has no unique ID and advertises as "Tempo-BT-unprovisioned". These appear in the system when detected by Bluetooth scanning, so the admin knows a new device is available to initialize.

- Provisioned (Unassigned): Device has a unique Bluetooth ID set, and a Bluetooth name "Tempo-BT-xxxx" (with the last 4 hex of ID), but it’s not yet assigned to any user in the app. It will show up on the device list as available to assign.

- Assigned: Device is provisioned and tied to a specific user (owner). The device’s internal storage (e.g., on its SD card) contains a uinfo.json (user info) file with the assigned user’s UUID and a “nextJumpNumber”. The nextJumpNumber is used to label the jump count on each new log file (so the device increments it as logs are created).

In the DB, we also track which user is the current owner, and we also have fields for lending (if lent out, to whom, and lending mode), as described in the lending section.

Continuous Device Discovery (Bluetooth Service): The bluetoothScanner worker continuously scans for BLE advertisements to maintain a live device list. Implementation:

It uses a scanning API (BlueZ via DBus or simply periodic mcumgr calls to scan) to find devices advertising the name pattern "Tempo-BT*".

If it sees "Tempo-BT-unprovisioned", it knows a brand new device is around. The service will create a Device entry in the DB if not already present, with state Unprovisioned, and note its temporary BLE ID (address) so that an admin can act on it. Possibly, it can auto-generate a placeholder unique ID for it in the DB until actual provisioning sets one.

If it sees "Tempo-BT-xxxx", it can derive or look up which device this is (perhaps store Bluetooth address in the DB to recognize devices, or the “xxxx” suffix could correlate to something). Each device has a hardware UID as well (maybe the MCU’s ID); provisioning likely involves writing that to device and DB.

The service updates each device’s lastSeen time each scan. Devices seen in the last 5 minutes are flagged online; others offline. The 5 minute threshold corresponds to DISCOVERY_WINDOW default of 300 seconds. This value can be tuned in config.

For any device that transitions from offline to online (not seen for >5min, now seen), the service triggers a log fetch routine. It connects to the device (using the Bluetooth lib) and queries the list of log files stored on the device. The device likely keeps logs as files named by jump number or timestamp. The service compares these with what’s been uploaded (it may maintain a local cache or ask the DB for known log hashes for that device). For each new file found, it uses the mcumgr file transfer command to download it to the server. This might be done via a temporary local directory or streamed directly into the database. After transfer, a new JumpLog record is created with the file’s data. The User ID metadata from the device is used to associate the log with the correct user in our DB. In practice, this means the device’s uinfo.json contains the user’s UUID; the service reads that (likely via a shell command or special GATT characteristic) to know the owner’s ID. (If device firmware doesn’t yet have that, as the note suggests, an alternative is that we assume whichever user it’s assigned to in DB is the owner. Going forward, firmware will support querying the stored user UUID).

Once logs are uploaded successfully, the service could optionally remove them from the device or mark them as uploaded (to avoid duplicate uploads). However, the requirements did not specify deletion on device, just not re-uploading known files. So the service likely just tracks file names or hashes to skip already uploaded ones.

This process happens seamlessly in the background. The user just turns on their device after landing, and within moments the log is harvested. If multiple devices come online together (formation landings), the service handles each in turn. The device might signal readiness by advertising some flag; otherwise the periodic approach covers it.

The scanner ensures the device list in DB is up to date: it adds new records for unprovisioned devices, updates known devices’ status, and could even remove devices not seen in a very long time (though likely we keep them in DB indefinitely unless explicitly removed).

Bluetooth Library Usage: The scanner worker uses the services/bluetooth (TS BT class). It might call something like bluetooth.scanForDevices(timeout) and get a list, then for each new device bluetooth.connect(deviceAddr) and then bluetooth.readLogList() etc. The bluetooth module wraps the details of calling mcumgr commands:

mcumgr likely has commands like mcumgr fs ls /logs to list files, mcumgr fs read /logs/log123.bin to download, and perhaps a shell command to read the uinfo.json.

The library will parse the output and return structured data to our service.

Error handling: if a transfer fails mid-way (common in BLE), the library should retry automatically. If ultimately failing, the service might log an error (perhaps in initialAnalysisMessage field of a placeholder log or an admin alert). The system can try again on next detection.

Device Initialization via Shell (Factory Setup): The scanner or a separate admin action can handle initial provisioning of an unprovisioned device. Possibly, when an Admin chooses “Initialize” from the UI on an Unprovisioned device, the backend uses the Bluetooth library to assign a new Bluetooth ID to it (if not already assigned) and give it a proper name. The requirement mentions that on provisioning completion, the device flashes blue for 10s. That suggests the library would send a command that triggers that flash (maybe part of the provisioning routine or a separate “identify” command).

The actual unique ID assignment might involve writing to the device’s non-volatile memory via a special command. Since details are TBD (“will be decided later” on how ID allocated), architecture-wise we allow for an initializeDevice(deviceId) function that picks an ID (perhaps the DB primary key or another sequence) and sends it to the device, then updates the DB record with that BT ID and name. After this, the device would switch its BLE advertising name from "unprovisioned" to the new "Tempo-BT-xxxx".

We also mark the device as Provisioned in DB so it won’t appear as unprovisioned anymore.

Admin Device Management UI: The /devices page lists devices with key info: Name (like "Tempo-BT-AB12"), Owner (if assigned), State, Online status, and a menu of actions. For each device, the actions are:

Assign to User: Shown for Provisioned devices (no owner yet) or to reassign an already assigned device. Admin chooses a user from a dropdown (list of users). The API call triggers deviceService.assignToUser(device, user, [nextJump]). Under the hood:

If the device was unprovisioned, perhaps this action also covers setting the unique ID first. But more likely we ensure a device is provisioned (given an ID) before assignment. Unprovisioned devices might show an “Initialize” first.

The service connects to the device via BLE, writes the new user’s UUID into uinfo.json along with the provided next jump number. If no jump number provided, default 1 (meaning this will be user’s jump #1 on this device).

Device acknowledges, perhaps flashes blue as confirmation (the device itself does the blue flash).

Service updates DB: set device.ownerId = that user, state = Assigned, update nextJumpNumber if we track it server-side too.

After assignment, the device when turned on will log under the new user’s ID, and the scanner will attribute logs accordingly.

Initialize: Possibly this action is for unprovisioned devices, performing the unique ID assignment as described. After this, the device remains Provisioned (still no user) but now has an identity. We update DB accordingly.

Unprovision: For an already provisioned device (maybe assigned or not) that we want to reset to factory state. This would remove its owner assignment both on device and in DB. Implementation: connect to device, wipe its stored user info (or send a command to factory reset its settings). In DB, either mark it as Unprovisioned or delete the device entry (though probably keep it but flag as unprovisioned so it can be re-initialized).

Blink: This sends a command to the device to blink its LED (amber flash for 10s) to physically identify it. No DB change, just a Bluetooth action. Useful when multiple devices are on and you need to pick one.

These actions require the device to be online (within range and powered) to succeed. The UI will likely disable or gray out actions for offline devices. The requirement explicitly states devices must be online for assign or unprovision. The scanner’s lastSeen timestamp tells us if it’s online (within 5 min).

If an admin attempts an action and the device has just gone out of range, the BLE command will fail; the backend should handle that gracefully and return an error message (UI can show “Device not reachable, please ensure it’s powered on”).

Integration with Lending: When a user (non-admin) lends a device via /lend page, under the hood it likely uses some of the same functions. For example, lending to an existing user essentially is a temporary reassignment. The system doesn’t require an Admin for that because it’s the owner doing it for their device. The lendingService might call a similar routine to write a new user ID to device. We allow that because the owner is initiating it (and presumably the app permits device owners to reassign their own device to someone else as part of lending). However, to avoid giving regular users full device management UI, we encapsulate this in the lending workflow with limited scope (only their devices, and it records a lending status).

Device Data Flow: Once devices are assigned, the continuous scanning and log upload take over. Each device’s logs always carry the user ID it’s assigned to on device, ensuring the logs go to the right user in the app. If a device is lent, that assignment changes on device for the duration of lending so logs go to the proxy or other user’s account. After lending, assignment goes back.

BlueZ vs mcumgr: The architecture leaves room to switch the underlying Bluetooth implementation. We default to using the mcumgr CLI (since a TS library/wrapper is provided for it). This requires the Linux BlueZ stack and the mcumgr tool installed on the Pi. If direct BlueZ TS integration becomes viable, the services/bluetooth could use that instead, but it’s an internal detail—either way, the deviceService and scanner use the same interface.

In summary, device management is handled by a combination of real-time background detection and admin-triggered commands. The Bluetooth interface abstracts the complexity of talking to hardware, so the higher-level services just call methods to get logs or set user IDs. The database keeps track of state changes, and the UI provides administrators with full control while giving regular users limited, guided interactions (like lending). This design meets the requirements of provisioning freshly flashed devices, assigning them to users, keeping track of online/offline status, and performing special actions like blink for identification.

## Jump Log Collection and Processing

The primary function of Tempo Insights is to collect jump logs and turn raw data into useful information. This involves capturing logs from devices, storing them, processing them, and then making them available for user review.

Log Upload and Storage: As described, the Bluetooth scanning service automatically uploads new jump logs from devices when they come into range after a jump. Each log file (on the device it might be e.g. a .bin or .csv) is saved in the JumpLog table as a binary blob. Storing logs in the database (Supabase) is acceptable for our size needs; typical log sizes (estimated up to ~8MB) and frequency are manageable on a local Postgres. Each log entry includes:

A unique ID (UUID).

The owning user’s ID (from device metadata or assignment).

The device ID it came from.

The jump number (the device likely generates this sequentially, but to double-check, we could compute: if the user’s profile keeps a total jump count, but since multiple devices could be used by one user, “jump number” might refer to the device’s count. The requirements imply the device tracks its own jump count (nextJumpNumber). We might store both the device’s jump count and perhaps also compute an overall count per user if needed).

Timestamp of log start (likely when recording began on the plane). If the device logs absolute time or at least date, we store that. Otherwise, we mark it and later during analysis we approximate the exit time and hence the date/time of jump.

The raw file blob and a SHA-256 hash (to ensure integrity and uniqueness).

A default note (empty initially, user can edit to add notes about the jump).

visibleToConnections flag defaulted to True.

Initially, fields like exit altitude, freefall time, etc., are unknown; they will be filled by the analysis worker.

initialAnalysisTimestamp null (flagging it for processing).

The upload process for each file is atomic – if an upload fails mid-way, the service will retry. Only when a complete file is stored and verified (hash match) will it finalize the DB record. The hash also prevents duplicates (if somehow the same log is attempted twice, we can detect it by hash and skip or update).

Preliminary Analysis (Computed Fields): The log processing worker enriches the JumpLog records with computed information:

It sets the estimated exit time offset (in seconds from start).

Parachute deployment/activation times (also seconds from start).

Landing time offset.

Estimated exit timestamp (UTC) by adding the offset to the start timestamp (if the device recorded absolute time at start, or using device time plus possibly an offset).

Exit location (latitude/longitude) if available – some devices might have GPS data at exit; we’ll extract the first valid GPS point after exit.

These values are saved in the JumpLog row. They enable quick display of summary info (e.g., freefall duration = deployment time - exit time, computed on the fly).

Formation Detection: As explained, the worker groups logs into FormationSkydive records when multiple logs have closely timed starts. The FormationSkydive model holds references to each JumpLog in that jump and identifies the participants. We ensure that if a user has set their log to private (visibleToConnections = False), that log can still be grouped (so the user knows they were part of a formation), but other users in the formation will not see that user’s data unless they make it visible. The formation grouping occurs progressively: if Jump A is uploaded and finds Jump B from 60 seconds apart, it creates a formation linking A and B. Later, if Jump C comes in that is 90 seconds from A (and say 30 from B), it gets added to the same formation. The formation’s base jumper is initially set (maybe always the first log’s user, or perhaps the one with lowest exit altitude as a guess for base; anyway, it can be changed by users later). Participants can go to the formation review page to see all involved.

### Post-Analysis Use: Once analysis is done, users can see much richer information:

On their Home dashboard, the summary panel for a jump can show “Exit alt: X ft, Deployment alt: Y ft, Freefall: Z seconds, Avg fall rate: W mph” for that jump. These come directly from the analysis fields.

The list of “Formation Jumps” on the side can highlight if any new formation events occurred. E.g., “Jump #15 – 3-way formation with [Alice, Bob, You]”. Clicking it opens the formation review page.

For solo jumps, the user might have an option to view more details (like a chart of altitude vs time). That could be a separate page or modal – possibly clicking a jump in “My Jumps” list could open a dedicated jump details page or overlay containing a Recharts graph of that jump’s altitude and vertical speed over time, along with any notes and the ability to edit the note or toggle visibility.

### Recharts for 2D Graphs: We incorporate Recharts for plotting charts like:

Altitude or Vertical Speed vs Time: A simple line chart or area chart showing the profile of the jump. This can be displayed on a jump’s detail view. We can also overlay key events (exit, deployment, landing) as markers on the chart for clarity.

Fall Rate Baseline Analysis: For a jump that was a “fall rate baseline” dive (the special alternating fast/slow dive described), we can plot the normalized fall rate over time. The analysis would yield a series of data points (time vs normalized fall rate). Recharts can plot this and we can highlight the portions identified as “slowest stable” and “fastest stable” fall rates. Additionally, draw horizontal bands at the “average jumper” fall rate range (e.g., 110–130 mph normalized) for comparison. This gives the user a visual sense of where they lie relative to average. This chart might appear on the jump’s detail page or a dedicated “Performance” page.

Recharts was chosen for its ease of integration into React for these standard charts, as opposed to D3 which we reserve for the complex visualization. This separation of concerns allows developers/designers to use high-level chart components for simple graphs, speeding up development.

Data Normalization (Fall Rate): The concept of normalized fall rate is handled in the analysis code. The worker, when it identifies a baseline dive or even for general jumps, can calculate the fall rate (ft per second or similar) for each interval and adjust it for air density using the standard atmosphere formula. This requires knowing altitude (from pressure) and possibly temperature/humidity if available (likely not, so assume standard ISA conditions or use an approximate model). The output is a “calibrated” fall rate that answers “how fast would the jumper be falling if the air density were at sea level.” This removes altitude as a factor.

For each time slice of freefall, the worker can compute this normalized fall rate. It might not store every data point (that would be a lot of data); instead, it might compute summary metrics: the peak and minimum normalized fall rates achieved. Those are the values to compare to the typical 110–130 mph range. We likely store those two numbers per jump if the jump is flagged as a baseline test. Alternatively, we mark the jump as baseline type and store the full time series in a separate table or as JSON. But a simpler approach: compute min/max normalized in code when needed.

The Formation review page’s base info shows “fall rate; calibrated fall rate” for the base at any given moment. How do we get that in real-time? Possibly, we compute the base’s current normalized fall rate during playback by applying the formula to their altitude at that frame (which we know) relative to base (though base altitude in formation frame is relative, but we know base’s actual alt above ground from data, so can recompute density). This dynamic calculation can be done in the D3 component if needed, or precomputed array for base’s normalized fall rate over time.

The system’s config allows tuning the “average jumper” range (min/max) as mentioned earlier. These values (110, 130 mph default) would be stored either in a constants file or possibly a database table (since marked tunable as system settings). An administrator interface could be provided in the future to change them, but for now, a simple config constant is fine.

Quality and Error Handling: If the analysis worker encounters an unexpected condition (e.g., a very short log, or missing data), it can record a message in initialAnalysisMessage. The UI (maybe on the jump detail) can surface that to the user or to admins for debugging. For example, “GPS data unavailable for this jump, cannot compute exit location” or “Log corrupted or incomplete”. This helps in troubleshooting device issues.

By structuring log collection and analysis this way, the heavy lifting is done server-side asynchronously, providing users with near real-time results (typically within 30 seconds of upload, given the worker cycle). The division between upload and analysis means we could scale those independently if needed (though on a single Pi, one instance of each is fine). The architecture ensures that each new piece of data flows through: device -> log upload -> DB -> analysis -> DB -> UI, with each step decoupled but coordinated through the central database.

## Formation Skydive Visualization (D3)

One of the standout features of Tempo Insights is the interactive 3D visualization of formation skydives for debriefing purposes. This is implemented using D3.js for lower-level control over SVG/canvas rendering and possibly a bit of three.js or custom math for 3D coordinate handling. Here is how this feature is architected:

Data Preparation: When a formation skydive is detected and created by the analysis worker, each jump log in the formation will have time-series data of the jumper’s position. The device logs likely contain:

GPS coordinates (lat, lon) at some frequency (maybe 1 Hz or more).

Altitude information (from the barometric sensor, giving altitude above MSL or above ground if calibrated).

Possibly orientation or other data, but for our purposes we need position vs time.

The analysis worker might not fully process the time series (as that can be a lot of data), but it could trim or resample it if needed. The formation review page will need these time-series for all participants. There are two approaches:

Compute on the fly in the browser: The raw log blobs could be sent to the client and then parsed in JS to extract positions. This might be heavy for large logs, and not very efficient on the Pi or network.

Precompute data server-side: The analysis worker could generate a simplified trajectory for each jump and store it in the database (for instance, as an array of coordinates with timestamps, possibly reduced in resolution for performance). This could be in a separate table or even precomputed JSON stored in a column. Given the Pi’s limited resources, precomputing when possible is wise.

Considering complexity, a middle ground: The server can compute key points or let the client do it for now. For MVP, it might be acceptable to fetch the entire log (which could be a few MB) and let client parse it using JS, but ideally, the worker populates a FormationParticipant table with something like { formationId, userId, positions: [ {t:0,x:...,y:...,z:...}, ... ] }. The x,y,z would be relative positions computed with a chosen base or with absolute frame that can be rotated. However, since base is user-selectable, it might be easier to store absolute positions (like ECEF or lat/lon/alt) and compute relative in client when needed.

Coordinate System Transformation: The requirement defines a Formation Frame: +X axis is along the aircraft’s forward ground track at exit, +Z is up, +Y is chosen to complete a right-hand system (essentially lateral axis). How do we determine the aircraft’s direction? Usually, if two or more jumpers exit, their initial horizontal separation gives a hint of the line of flight. If only one jumper (solo), there is no formation, so no viz needed. With at least two, one could take the vector between exit points of base and another as an approximation of the line of flight direction, or if plane GPS was known it’d be exact. The system likely assumes the first two jumps in a formation define the plane’s path (they mention formation date is derived from first two jumps’ times, maybe similarly, first two positions define heading).

We can take the base jumper’s exit coordinate as origin (0,0,0). Compute the vector from base’s exit point to another jumper’s exit point (projected on horizontal plane). That vector could define the +X axis direction. Normalize it. Then define +Z as upward (pointing from Earth’s center, but since altitude differences are small area, we can approximate up as vertical). +Y then is perpendicular to both to form right-hand coordinate (essentially, if +X is east for example and +Z is up, +Y would be south or north depending, ensuring a right-hand basis).

Alternatively, if only one other jumper, that vector is fine; if multiple, maybe average headings or just use base’s plane direction sensor if known (some devices might log plane heading if started before exit, but likely not).

The transformation of lat/lon to X/Y: we likely convert lat/lon differences to local flat coordinates (e.g., using a simple equirectangular projection centered at base’s exit or a proper UTM projection for more accuracy). Since distances are small (within a few miles at most), a planar approximation is okay. Each point’s altitude difference from base’s exit altitude gives Z coordinate.

We then have each jumper’s trajectory in these relative coordinates. At exit moment, base is (0,0,0). Others at exit have coordinates (x,y,z) relative to base (z likely 0 at exit if we consider all exited at nearly same time; if one exits later, our timeline will handle that by starting their motion a few seconds offset).

Visualization Implementation: Using D3, we can create either an SVG or Canvas element to draw the jumpers as moving points:

Each jumper could be represented by a colored dot or icon (maybe with their initials or a number). The base jumper might be highlighted.

We need to animate their movement from exit to landing. This requires iterating over time steps. We define a timeline from t=0 (formation start, roughly the earliest exit among them) to t = end (when last lands).

D3 can be used for its transition and interpolation utilities. However, since we need interactive control (pause, scrub, etc.), it might be easier to handle the animation loop manually (requestAnimationFrame or setInterval, updating positions each frame).

D3 would be helpful for scaling and axes if needed, but here we have an arbitrary coordinate space. We might still use D3 for ease of DOM manipulation or an d3.timer for animation.

We provide controls: play/pause, fast-forward (maybe 2x speed), rewind. On user input, we start/stop the animation loop or adjust a time index.

View Controls: We have three preset camera angles: top-down (“god’s eye”), side view (from -X, meaning looking from behind the plane’s exit direction), and free rotate. To implement rotation, we can allow click-drag to rotate the scene (this is essentially a 3D rotation of the coordinates or camera). We might not use a true 3D engine, but rather compute 2D projections of 3D points ourselves. For example, treat the formation frame as a 3D space and apply a rotation matrix based on user input angles, then project onto 2D for drawing. D3 can handle matrix transforms or we can do it manually. Given moderate complexity, a simpler route: use three.js (a minimal scene with points) to handle 3D rotation and perspective. But since the requirement specifically says d3 for 3D, we likely stick to custom D3 math.

For the preset angles:

God’s eye: simply ignore one coordinate (Z or effectively set camera looking from above, so we plot X vs Y only, perhaps with altitude indicated by color or an icon trailing line).

Side (-X): ignore Y (or compress Y), plotting Z vs X from that perspective.

Free rotate: apply yaw and pitch rotations to all points.

Because this is complex, we might restrict MVP to the preset views or limited rotation to ensure we can deliver it.

Legends and Data Display: The visualization includes a legend showing the base’s fall rate and altitude in real time. Implementation:

We know base’s data over time. We can display base’s current actual fall rate (vertical speed) in mph, and their altitude above ground (AGL). If device logs altitude above ground, use that; else compute AGL by subtracting exit altitude or using some ground elevation reference. Likely device knows ground level from pressure calibration at takeoff or user input.

“Calibrated fall rate” could also be shown (perhaps in the base info panel as per UI spec). That would be the normalized fall rate at the current time. We can calculate it by taking base’s current vertical speed and adjusting for density ratio at their altitude (precomputed or formula).

The right side “Base Info” panel thus might show: “Base: [Name]; Fall rate: 120 mph (normalized 125 mph); Altitude: 5000 ft AGL”. We update these as the animation runs.

Also on the right side, they want a jumper list with distances and closure rates relative to base. This means for each other jumper at the current time, we calculate:

Horizontal distance from base (in feet or meters).

Vertical separation (which might be clear from altitude difference, but they specifically said distance from base, likely meaning 3D distance or horizontal? “distance from base (feet)” might mean horizontal distance).

Closure rate: the relative speed towards/away from base, given in fps and mph. We can get this by computing the derivative of distance between that jumper and base over time (or using velocity vectors if available). This is dynamic, updating as they move.

These calculations happen in real-time in the D3 component every frame (which is fine for a handful of jumpers).

The UI likely highlights if a jumper is getting dangerously far or fast (maybe using the color palette’s alert color #f04848 for unsafe values).

Performance considerations: The Pi 5 is fairly powerful, but the visualization will run in the browser (which could be on the Pi or any device connecting to the web app). The heavy computation (distance math, etc.) is not huge for, say, up to 8 jumpers over 60 seconds at 1Hz (480 data points). The bigger constraint is probably smooth rendering at, say, 60fps. But we can decouple simulation time step from rendering frame rate if needed or simply accept a slightly slower frame rate. Using canvas might be more efficient than SVG for continually redrawing moving points, so likely we use D3 with Canvas (d3 can render to canvas by binding data to drawing calls instead of DOM). Another trick: precompute trajectories and maybe convert them to pixel positions for each frame of the animation at load time. But since rotation is dynamic, that has to be done on the fly.

User Interaction: Users can play/pause, scrub the timeline (maybe a slider that they can drag to jump to a specific time), and change the viewpoint. The interface should be intuitive for debriefing scenarios, possibly full-screen capable.

Integration in App: The formation review page fetches the formation data by its ID (via getServerSideProps or an API call). The server returns the list of participants, their names, maybe their color assignments, and either their trajectory data or endpoints to fetch it. We then initialize the D3 component with that data. The rest is client-side. If needed, the page can dynamically load additional data (for example, if we didn’t precompute trajectories, it might request each participant’s raw log and parse it — but as noted, precompute or storing processed coordinates is preferable).

This feature is complex, but it directly addresses the need for a group jump debrief tool that current solutions lack. By carefully computing positions and using D3 for the rendering, we provide a “movie” of the jump that can be manipulated in real time. The choice of D3 is justified by the custom nature of the visualization (three.js or game engines might overshoot the needs). It also allows integration of the data-driven approach (e.g., easily binding data to multiple textual readouts like distances, speeds in the legend, syncing with the animation). The color palette will be used here as well: e.g., using color-3 #ddff55 or color-9 #855bf0 for emphasis lines or highlighting trajectories, color-8 #f04848 to flag “unsafe” conditions (maybe if a jumper is falling significantly faster/slower than base, or any warnings).

## Device Lending Workflow and Proxy Users

The device lending feature introduces a multi-user interaction with devices and data, requiring careful permission handling in the architecture. As previously touched on, here’s a focused view on how lending is handled:

Initiating a Loan (Lend My Device): On the user’s home page, there is a "Lend My Device" option. This leads to a lending form (/lend page or a modal) where the user (must be the owner of at least one device) selects:

The device to lend (from a dropdown of devices they own).

Who to lend to. This is a search field to find an existing user by name or email. If the person does not have an account, the user can choose to create a new proxy user on the spot. The UI might have an autocomplete: as you type a name, it shows matching users, plus an option "Add as new user..." which triggers input of full name for a proxy.

The duration of the loan: “one jump”, “any number of jumps”, or “until I reclaim”. For MVP, "any number" and "until reclaimed" likely function the same (indefinite until manual reclaim), but we preserve the option in the data.

Lending API and Data Changes: When the form is submitted:

If a new person was specified, the lendDevice service creates a Proxy User in the Users table with the provided name and isProxy=true. It records the proxyCreatorId as the current user (lender). The new proxy user gets a UUID like any user and a slug (e.g., from their name).

The device’s record is updated: ownerId remains the original owner (we do not change ownership on loan; the concept is the owner keeps ownership but just lends out). We set lentToId to the target user (proxy or existing). We also set fields for lending: e.g., lendingExpiresAfter = 1 jump (if one jump), or a flag for until reclaim. We might also store the time the lending started.

Immediately upon lending, the service should also re-provision the device to the borrower. This is crucial: it ensures the next log file generated by the device will be tagged with the borrower’s user ID, not the owner’s. Implementation: use deviceService.assignToUser(device, borrower) similar to admin assignment, but note we are not transferring permanent ownership, just a temporary assignment. The device’s uinfo.json is updated to the new user’s UUID and the next jump number likely reset to 1 or kept – maybe it continues from where the device left off, but since it's a different user, typically jump numbering is per user. To avoid confusion, it might start at 1 for the new user (on that device) or continue; the requirement doesn’t specify resetting jump number on lend. Possibly better to continue device jump count globally but that means the jump number field in JumpLog is not strictly the user’s personal jump count (since that user’s first jump on a borrowed device might have jumpNumber=50 if the device had 49 jumps prior). However, JumpLog jumpNumber was described as “the integer Jump Number (automatically assigned)”, likely meaning each user’s personal jump counter in app. So maybe on upload, we override the device’s internal count with the user’s own jump count. That’s another interpretation: The system could track each user’s jump count and assign the Jump Number sequentially at app level, ignoring device’s number. That would make more sense for personal logbook continuity. If so, the device’s nextJumpNumber is more for the device to label logs, but the server might not use it for user’s global count. In any case, lending doesn’t break anything: logs will come in tagged to the correct user, and the server will assign jump numbers in sequence for that user’s record.

The API returns success; the UI might show a confirmation like “Device lent to X. It will automatically return after one jump.” or “Remember to reclaim your device later.”

During the Loan: The borrower (if an existing user) can log into the app and will see the jump appear in their account once they make the jump. If they are a proxy user, they don’t have login credentials yet, but the data is still collected under their proxy account. The original owner should be able to see the proxy user in their connections (since they created it) and likely see that proxy’s jumps (because the proxy’s data is visible to the creator by definition – the requirement says proxy and their data are visible to the user who created them, akin to a dependent account).

If the owner wants to monitor the jump in real-time, they could see the device is online but the log will go to proxy’s account. The UI might not surface proxy’s current activity, but after upload, the owner might see the log either via knowing the proxy user or via the device context. Possibly, for usability, the system could notify the owner: “Your device (id) uploaded a jump for [Proxy Name].” That could be in a notification section.

Auto-Return for One Jump Loans: If the loan was one-time, we implement auto-reclaim:

The simplest trigger: the Bluetooth scanner after uploading a log can check the device’s lentToId and lendingPolicy. If policy = one_jump, then upon successful upload of one new log, the scanner (or logService) will:

Update the device record: clear lentToId (set to null) and clear the lending flags (back to normal assigned state).

Optionally, automatically reassign the device back to the owner on the device itself. This means writing the original owner’s user UUID back to uinfo.json. The scanner can perform that by invoking the same provisioning function. However, performing a provisioning operation via the scanner might need the scanner to pause scanning and do this command. It’s feasible to integrate – after finishing the file upload, the same connection can be reused to send the assign command for the owner. Or we might queue a task for the deviceService to do it. But doing it immediately in scanner is logical.

If not done automatically on backend, we could alternatively notify the owner that “Loan period (one jump) is over, please reclaim device”. But automation is preferred (and mentioned as possible with “automatically return” using the stored info).

If the loan is “until reclaimed” (indefinite), the device remains assigned to the other user until the owner manually triggers reclaim. That would be another action – perhaps on the home page or devices page, the owner sees the device marked as “Lent to X (until reclaim)” with a button “Reclaim now”. Clicking that triggers essentially an assign back to self. Only the owner (or an admin) should be allowed to reclaim.

During an ongoing loan, the device is effectively treated as belonging to the borrower (for data visibility). The borrower could use it for multiple jumps if allowed. The owner’s jump count won’t increment during that time because they’re not jumping with it. The owner still remains the permanent owner in DB for reference.

### Permissions and UI considerations:

A user should not be able to lend a device they don’t own. The lending page only lists their devices.

A user cannot lend a device that’s currently lent by someone else to them (since you only see your own devices to lend).

Admins likely do not interfere with lending except possibly to see device statuses. The device list could show that device X is “Lent to Y by Z” so admins have oversight.

If a device is lent to a full user account (not proxy), that user effectively has an assigned device temporarily. We might reflect that on their profile (like “Borrowing device from [Owner]”).

While borrowed, that user might appear as the device owner in some contexts (for instance, scanner might treat them as owner for log attribution). But we do maintain original owner in DB. We just use lentTo as the active user. This dual tracking prevents confusion of ultimate ownership vs current user.

### Account Claiming (Proxy to Full User):

This was discussed: the lender generates a claim link (perhaps from a list of proxies they have). Implementation wise, we have a UserInvitation model with fields: id, proxyUserId, createdAt, expiresAt, maybe an invitationCode (random string) if we don’t want to expose sequential IDs.

The accept-invitation/[id] page retrieves that invite record, ensures it’s valid (not expired, not already used), and then allows setting email/password for that proxy user. On form submit, authService.completeProxyRegistration(proxyId, email, password) is called. It checks again that the user is still a proxy and not already claimed, then updates the record. After that, the user can log in normally.

Possibly we should send a confirmation email or something after claim, but not required now.

The UI could automatically log them in or direct to login after success.

The lending feature essentially extends the system to handle multi-user scenarios around devices while preserving data integrity. We create pseudo-accounts (proxies) to store data for non-registered jumpers, ensuring no data is lost and that if they join later, their history is intact. The architecture segment that deals with this is mainly in the lendingService and related API routes, with a bit of involvement from the Bluetooth functions (to reassign device IDs). By planning the state fields (owner vs lentTo) and the workflow (especially auto-return logic), we ensure a smooth experience: a device owner can confidently lend out a device and get it back (in data terms) without admin intervention, and a new user can transition to a full account seamlessly to access their own jumps.

## Theming and Styling Strategy

Tempo Insights is built with a consistent dark theme across the application, using Mantine for styling. The approach to theming is as follows:

Mantine Provider and Global Theme: In _app.tsx, the app is wrapped with <MantineProvider theme={customTheme} colorScheme="dark">...</MantineProvider>. We set colorScheme="dark" to enable Mantine’s dark mode styles globally. The customTheme extends Mantine’s default theme with our specific colors:

We define a palette using the given colors. For example:

primaryColor could be set to a custom color name “brand” which maps to #ddff55 (the bright green) as that seems intended for emphasis text/lines. Or we could set Mantine’s default primary to one of those if appropriate.

background colors: set Mantine’s colors.dark[7] or similar to #002233 for the app background (very dark blue), and use #11425d for secondary background surfaces (cards, panels). Mantine’s theme allows overriding colors array for custom shades.

Text color: ensure the high contrast text color is #c5c0c9 for regular text on dark background. Lighter text (heading maybe) could use #f6f2e8 for contrast.

Highlight/alert color: use #f04848 for any warnings or critical highlights (Mantine’s default red could be replaced by this).

An alternate accent (maybe used in charts or secondary highlights) is #855bf0 (a purple).

We will leverage Mantine’s theming to apply these globally so that all built-in components like buttons, modals, etc., adhere to the scheme. For instance, we can set primaryColor: 'brand' and define 'brand': ['#f6f2e8', ... , '#ddff55'] as a color scale if needed, where #ddff55 might serve as a primary shade for buttons or links, giving a neon contrast on dark.

The Mantine default fonts will be used as specified, meaning it will choose system fonts (e.g., -apple-system, BlinkMacSystemFont, ... on Mac/iOS, etc.) unless we override. We likely don’t need a custom font at this stage, ensuring good readability.

We can also globally apply styles like setting body background to color-1 (#002233).

Component-Level Styling: Mantine allows styling via props and default styles. We will use Mantine components predominantly, which come pre-styled for dark mode (e.g., <Table>, <Card>, <Button> adapt to dark theme). For any custom components, we will use Mantine’s style hooks or CSS-in-JS to keep the style consistent. We avoid Tailwind as per requirements.

For example, in charts (Recharts), which are SVG-based, we will manually apply our theme colors to axes, labels, etc. The text color in charts should be the light text color (e.g., #c5c0c9) for readability on dark background. Lines for data series could use accent colors (like one series in green #ddff55, another in purple #855bf0). Grid lines might be a dark gray closer to background.

The D3 visualization will also use theme colors: e.g., base jumper could be rendered in the primary/emphasis color (#ddff55) so they stand out. Others can use color-4 or color-9 for differentiation. The ground or any reference lines could be drawn in a muted color that blends with background or secondary color. Using these colors not only meets branding but also ensures high contrast on dark (for instance, #ddff55 is a bright lime that will pop against near-black).

Interactive states (hover, active) on components will also be derived from these palettes via Mantine. Mantine’s default dark mode styling will be tweaked minimally to incorporate our specific brand colors.

Responsive Design: Mantine provides grid and flex utilities. We’ll ensure the layout works on typical devices. Since this is a web app possibly accessed on tablets or laptops at the dropzone (and maybe phones for some features like viewing an account claim QR), we should ensure components scale. The theming covers color and font; responsiveness is handled in our JSX and CSS (e.g., using Mantine’s breakpoints to switch to a mobile-friendly view where needed).

No Tailwind or External CSS: We stick with Mantine’s styling system and maybe some custom CSS modules if needed for very custom stuff. But ideally, Mantine covers most. This avoids mixing styling paradigms and meets the “no Tailwind” stipulation.

Overall, the theming strategy is to present a professional dark interface that is consistent and uses the provided palette for visual identity. Dark backgrounds with light text reduce glare (useful in a sunny dropzone environment as well). The color choices also likely have good meaning: e.g., the bright green (#ddff55) might indicate safe/normal data, whereas red (#f04848) flags unsafe conditions (like an out-of-range fall rate). We will use these cues in UI components and data visualizations to convey information intuitively.

## Configuration and Constants Management

The application has several parameters that need to be configurable or easily adjustable, either at build time or run time. We handle configuration as follows:

Environment Variables: For deployment-specific secrets and config, we use a .env file. This includes the database connection string (Supabase URL and service key if needed, though with local Supabase we might just use a direct Postgres connection), the JWT secret for session tokens, and possibly a setting for DISCOVERY_WINDOW. If we want to allow adjusting the BLE scan window without code changes, we can put DISCOVERY_WINDOW=300 in the env. Similarly, the 30-day session expiry could be configurable, but it’s likely hardcoded since it’s a policy.

Constants Module: As mentioned, utils/constants.ts will hold various non-secret constants:

DISCOVERY_WINDOW (in seconds) – default 300s (5 min) as given. The Bluetooth scanner worker and the device online/offline logic use this. If we wanted to allow an admin to change it, we might later expose it in a settings UI that updates a database value, but for now it’s fine as a constant.

Fall Rate Norm Base Range: FALL_RATE_AVG_MIN = 110, FALL_RATE_AVG_MAX = 130 (mph). Marked tunable, but we can start with these defaults. We might add these to a settings table later; for now keep them in constants so that both front-end (to draw the comparison band on a chart) and back-end (to label someone above or below average) can use the same values.

Bluetooth settings: e.g., a BLE_SCAN_INTERVAL if we implement scanning in intervals, or MAX_CONCURRENT_TRANSFERS. Possibly the path to the mcumgr binary if it’s not in PATH, could be configured.

Analysis thresholds: The criteria for exit detection (2000 fpm for 1s), parachute activation decel (0.25g), etc., could be constants. If we foresee tweaking them often, having them in one place is useful.

Pagination or UI page sizes if any (like number of logs to show per page).

Dropzone Location: one thing not explicitly in requirements but possibly needed is the local time zone for displaying jump times (they mentioned local time zone for jump time). We might configure the server’s time zone or a specific offset if needed. But this can be gleaned from environment or simply use the system’s local tz since the server is on-site presumably.

Database-held Settings: For MVP, we might not implement a full settings UI. But the note "tunables as system settings" suggests possibly a future feature to adjust the fall rate comparison range. If that were a requirement now, we would create a table e.g. SystemSettings(key, value) and load those on startup or on demand. But since clarifications didn’t emphasize building an admin settings UI (and given only a few values, not worth the overhead now), we document that these can be changed in code or env. For example, if testing shows 120–140 mph is a better baseline range, a developer or admin editing a config file can change it.

Secrets and Security Config: As noted, no special encryption beyond standard practices. If in future Bluetooth pairing is added, keys for that might be needed. For now, we keep it simple.

Maintaining Config Synchronization: We must ensure that any config used by both the Next.js app and the workers is accessible to both. With our approach:

Environment variables are accessible to both (since workers can be spawned with the same env).

Constants file can be imported in both sets of code (if the workers are part of the same codebase, which they are, just separate entry points).

Alternatively, workers could read from a JSON config file or the database. But that’s overkill for a handful of static values.

By centralizing these, we avoid magic numbers spread across code, and it's clear where to adjust parameters like the scanning frequency or analysis thresholds.

Logging and Monitoring Config: Not exactly asked, but worth noting: we can have config for log levels (verbose logging for debugging the Bluetooth interactions, etc.). On a Pi, we might run these processes via something like PM2 or systemd, and logs would go to files. We should ensure sensitive data (like passwords) are not logged.

In summary, configuration is managed in a straightforward manner with .env and a constants module. Critical operational settings like time windows and thresholds are easily tweakable, supporting the need to fine-tune the system without deep code changes. This approach covers current needs and leaves room to evolve into a more dynamic settings management if required later.

## Integrated Services vs. External Processes

Finally, to clearly distinguish which parts of the system run within the Next.js web application and which run as independent processes:

Web Application (Next.js): The core web server hosts the Next.js application, which includes:

All pages (UI rendering, SSR where used, etc.).

The API routes that handle user requests (authentication, queries, admin actions). These routes execute within the Next.js runtime (Node.js server context).

The web app also directly integrates certain Bluetooth actions and other services when triggered by the user. For example, when an admin clicks “Assign device”, the API route will call the Bluetooth library to perform that action in real-time and return the result. This means the Next.js server process needs Bluetooth access for those on-demand tasks. Since it’s running on the same machine with Bluetooth hardware, that’s possible (with necessary Linux capabilities if needed to allow a user process to access Bluetooth).

Similarly, generating a claim QR code or zipping export data are done inside the web app process when requested.

Bluetooth Log Discovery Service (External): This is a separate Node.js process (could be started by a script or as a daemon). It is not part of the Next.js request/response lifecycle. It runs continuously from system startup, performing BLE scans and file transfers in the background. It interacts with the database (via Prisma or direct SQL) to record device info and logs, but it does not expose a web interface or API endpoints. Its only “communication” with the main app is through the database. This service might be packaged as part of the project (the workers/bluetoothScanner.ts we described), and deployed such that it’s launched alongside (or via) the Next.js app. On a Raspberry Pi with Ubuntu, one might use a systemd service for it or a process manager like PM2 to ensure it stays running.

It’s isolated so that any crashes or heavy Bluetooth operations don’t take down the web server. If it crashes, a supervisor restarts it.

The main web app doesn’t need to know if it’s currently running except that data appears when available. We do ensure only one instance runs, to avoid duplicate uploads.

Log Analysis Worker (External): This is another separate Node.js process or perhaps a scheduled job triggered by cron. The requirement suggests a continuously running process that wakes every 30 seconds, which is easiest as a simple while(true) loop with sleep. We can run this as its own script (e.g., node logProcessor.js) either manually or via a service manager. It reads and writes to the same DB as the web app.

Because it’s separate, it can be started/stopped independently (for example, an admin could disable the worker if needed for maintenance without shutting down the web UI).

If multiple instances accidentally run, we might process logs twice, but we can put a safeguard: e.g., a transaction to set a log as being processed. But given a single Pi deployment, we will run one instance.

We explicitly avoid integrating this into the Next.js API routes or using something like Next.js middleware for cron, because long-running tasks in Next’s context could block or complicate the server. Offloading to an external process is cleaner.

Database (Supabase): Although Supabase is an external service (it runs Postgres, possibly on the Pi or a local network server), in terms of our architecture, it’s the shared resource rather than a service we implement. Both the web app and the background processes connect to it. Supabase also offers an API and potentially triggers/functions, but we aren’t required to use those. We rely on Prisma within our Node processes for all DB access. So the DB can be thought of as part of the infrastructure layer.

## Summary of Responsibilities:

Web App Backend: Auth, serving pages, handling user-initiated actions (which includes some Bluetooth commands like provisioning), enforcing permissions, providing data to the UI.

Background Bluetooth Service: Autonomous device discovery and log ingestion, updating device statuses.

Background Analysis Worker: Post-processing of data and creation of derived insights (formations, stats).

These three operate concurrently and cooperatively.

This separation aligns with the clarification that the Bluetooth harvesting can run separate, while some device actions need to be in the web backend. It also adheres to the idea that no specialized real-time server (like WebSocket server) is needed since polling suffices, simplifying the integration – we don’t need to push from background to front-end directly, we just write to DB and front-end pulls.

Finally, we ensure that all these pieces can run on the Raspberry Pi 5 reliably. Node.js should be fine on that ARM architecture; Prisma will connect to the local Postgres; BlueZ is available on Ubuntu and mcumgr can be compiled or installed. We may need to run the Next.js server in production mode (next start with a built app) which typically is one process; our workers are separate Node processes started via separate scripts.

By clearly delineating which services are in-process vs out-of-process, we reduce complexity in the code (each part can be relatively independent) and we can scale or troubleshoot them individually. For example, if logs aren’t being processed, we look at the worker process without disturbing the web server. This architecture is modular and robust, fitting the needs of a small-scale but multi-functional system deployed on dedicated hardware.