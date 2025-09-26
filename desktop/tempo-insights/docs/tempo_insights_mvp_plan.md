# Tempo Insights MVP Build Plan

Each task is small, testable, and focused on one concern.  

---

## Phase 0 — Project & Environment

- **Task 1: Initialize repo**  
  Do: `pnpm dlx create-next-app@latest` (TS, App Router off, ESLint on)  
  Done when: repo builds and runs `next dev` without errors.

- **Task 2: Add core deps**  
  Do: add `@mantine/core @mantine/hooks @mantine/notifications recharts d3 prisma @prisma/client zod bcrypt jsonwebtoken`  
  Done when: `pnpm install` succeeds and `pnpm build` compiles.

- **Task 3: Project scripts**  
  Do: add scripts `dev`, `build`, `start`, `worker:bt`, `worker:analysis`, `prisma:generate`, `prisma:migrate`  
  Done when: `pnpm run` lists all scripts.

- **Task 4: Repo hygiene**  
  Do: add `.editorconfig`, `.nvmrc`, `.gitignore` (Node + Next + env + prisma)  
  Done when: tracked files are clean on `git status`.

- **Task 5: CI sanity (optional)**  
  Do: GitHub Action that runs `pnpm i`, `pnpm build`, `pnpm prisma generate`  
  Done when: CI green on main.

---

## Phase 1 — Theming & App Shell

- **Task 6: Mantine provider**  
  Wrap app in `<MantineProvider>` with `colorScheme="dark"`  
  Done when: any page renders Mantine styles.

- **Task 7: Theme palette**  
  Implement theme file with colors from spec; apply overrides.  
  Done when: background is `#002233` and text matches palette.

- **Task 8: Notifications provider**  
  Add `<Notifications />` top-level.  
  Done when: demo toast renders.

- **Task 9: App layout**  
  Base layout with left nav, center content, right sidebar.  
  Done when: three columns visible on `/home`.

---

## Phase 2 — Database & Prisma

- **Task 10: Prisma init**  
  Run `npx prisma init`, set `DATABASE_URL`.  
  Done when: `prisma generate` works.

- **Task 11: User & Role models**  
  Add `User`, `UserRole`.  
  Done when: migrate runs; tables exist.

- **Task 12: Device model**  
  Fields: ownerId, lentToId, state, bluetoothId, name, lastSeen.  
  Done when: migrate shows columns.

- **Task 13: JumpLog model**  
  Fields: rawLog, hash, offsets, flags.  
  Done when: migrate succeeds.

- **Task 14: Group & Membership models**  
  `Group`, `GroupMember`.  
  Done when: migrate succeeds.

- **Task 15: FormationSkydive models**  
  `FormationSkydive`, `FormationParticipant`.  
  Done when: migrate succeeds.

- **Task 16: Invitation model**  
  `UserInvitation`.  
  Done when: migrate succeeds.

- **Task 17: Seed admin**  
  Insert admin user.  
  Done when: seeding works.

---

## Phase 3 — Authentication

- **Task 18: Hash utility**  
  Implement bcrypt wrapper.  
  Done when: test passes.

- **Task 19: JWT config**  
  Env secret, issue 30-day JWT.  
  Done when: decode test OK.

- **Task 20: Login API**  
  `/api/auth/login`.  
  Done when: cookie set.

- **Task 21: Register API**  
  `/api/auth/register`.  
  Done when: new user logs in.

- **Task 22: Auth middleware**  
  Verify cookie + load user.  
  Done when: protected API denies anon.

- **Task 23: Role guard**  
  `requireAdmin`.  
  Done when: test blocks non-admin.

- **Task 24: Login page**  
  Mantine form.  
  Done when: redirects `/home`.

- **Task 25: Register page**  
  Auto-login + redirect.  
  Done when: works.

- **Task 26: Logout**  
  Clear cookie API.  
  Done when: `/home` redirects.

---

## Phase 4 — Navigation & Pages Skeletons

- **Task 27: Route guard HOC**  
  Redirects to `/login`.  
  Done when: `/home` requires login.

- **Task 28: /home skeleton**  
  Left nav + placeholders.  
  Done when: visible.

- **Task 29: /profile skeleton**  
  Editable profile form.  
  Done when: page loads.

- **Task 30: /devices skeleton (admin)**  
  List placeholder.  
  Done when: non-admin blocked.

- **Task 31: /users/[slug] skeleton**  
  User card.  
  Done when: seeded admin loads.

- **Task 32: /groups/[slug] skeleton**  
  Group info.  
  Done when: seeded group loads.

- **Task 33: /review/fs/[id] skeleton**  
  Canvas placeholder.  
  Done when: loads.

---

Phase 5 — Users & Groups Basics

- **Task 34: Slug generator**
  Do: human-name → URL slug; de-dupe with “-nnn”, also add suffix for any collision with fixed app URL top-level components (e.g., "profile", "api", "users", "groups", "home", "review", "login", "register", "devices")
  Done when: “Bill Jones” → “bill-jones”; collision → “bill-jones-123”.

- **Task 35: User profile read**
  Do: /api/users/me returns profile JSON
  Done when: /profile loads current data.

- **Task 36: Update profile**
  Do: /api/users/me PATCH (name, photo MIME + bytes)
  Done when: updating name/photo persists.

- **Task 37: Change password**
  Do: /api/users/password (old pwd verify, set new hash)
  Done when: new login works; old fails.

- **Task 38: Create group**
  Do: /api/groups POST (name, description, public/private)
  Done when: creator auto-member isAdmin=true.

- **Task 39: Group page read**
  Do: load group info + member list
  Done when: shows members with admin badge.

- **Task 40: Join public group**
  Do: /api/groups/[id]/join for public groups
  Done when: membership appears.

- **Task 41: Invite to private group**
  Do: /api/groups/[id]/invite (admin only) creates pending invite
  Done when: target user sees pending in “Pending Actions”.

- **Task 42: Accept/decline invite**
  Do: endpoints to accept/decline
  Done when: accept adds member; decline removes invite.

---

## Phase 6 — Devices (DB + Admin UI)

- **Task 43: Device list API**
  Do: /api/devices/list returns all devices + online status
  Done when: seeded records appear.

- **Task 44: Devices page list**
  Do: table with name, owner, state, online dot
  Done when: renders API data.

- **Task 45: Poll device status**
  Do: 15s polling with SWR/interval
  Done when: manual DB toggle reflects in UI within 15s.

- **Task 46: Assign modal**
  Do: admin UI to select user + optional nextJumpNumber
  Done when: selecting user POSTs to backend (stubbed action).

- **Task 47: Blink action**
  Do: admin UI kebab → Blink (stub)
  Done when: backend logs intent; UI shows toast.

- **Task 48: Unprovision action**
  Do: admin UI kebab → Unprovision (stub)
  Done when: device state flips to Unprovisioned in DB.

  (Bluetooth wire-up comes in Phase 8; for now, stub to DB changes so UI is testable.)

---

## Phase 7 — Jump Logs (CRUD + Lists)

- **Task 49: Create JumpLog API (internal)**
Do: /api/internal/jumps/create (protected to workers via shared token header)
Done when: posting bytes creates JumpLog row with hash.

- **Task 50: My Jumps list**
Do: /api/jumps/mine returns paginated jump summaries
Done when: right panel shows latest 5.

- **Task 51: Jump visibility toggle**
Do: endpoint to set visibleToConnections
Done when: toggle persists and affects visibility queries.

- **Task 52: Jump detail endpoint**
Do: /api/jumps/[id] (owner or authorized viewer) returns analysis fields & note
Done when: owner can GET; others depend on visibility rules.

- **Task 53: Edit jump note**
Do: PATCH notes (markdown) for owner
Done when: updated note renders on detail card.

---

## Phase 8 — Bluetooth Scanner Worker (ingestion stub → real)

- **Task 54: Worker skeleton**
Do: workers/bluetoothScanner.ts loop with sleep interval env DISCOVERY_WINDOW=300
Done when: logs “scan cycle” on interval.

- **Task 55: smpmgr presence check**
Do: on startup, verify `smpmgr` in PATH; log error if missing.  The plugin-path directory is the `smpmgr-extensions/plugins` directory in the source tree.  It will need to be part of the runtime directory structure and be referenced on the smpmgr command line when invoked.
Done when: prints version on success.

- **Task 56: Scan API wrapper (stub)**
Do: bluetooth.listTempoDevices() returns mocked devices for now
Done when: worker logs found devices.

- **Task 57: Update lastSeen**
Do: for each device, upsert in DB + set lastSeen=now(); mark online if seen within window
Done when: /api/devices/list shows online.

- **Task 58: New-file detection (stub)**
Do: maintain per-device uploaded filenames in DB (table DeviceFileIndex)
Done when: calling ingestion simulates one new file per appearance.

- **Task 59: Upload → JumpLog**
Do: write raw bytes (mock), compute SHA-256, set userId from device assignment
Done when: new JumpLog rows appear with hash.

- **Task 60: Replace stubs with smpmgr list**
Do: implement fs ls parse to list files on device
Done when: real devices list files.

- **Task 61: Replace stubs with smpmgr read**
Do: implement fs read to fetch bytes
Done when: real transfer stores bytes in DB.

- **Task 62: Idempotency**
Do: skip re-ingestion by filename or content hash
Done when: second scan does not duplicate rows.

- **Task 63: Device online/offline accuracy**
Do: transition to offline if not seen for > window
Done when: UI dot flips after timeout.

---

## Phase 9 — Analysis Worker

- **Task 64: Worker skeleton**
Do: workers/logProcessor.ts loop every 30s
Done when: logs cycle start/end.

- **Task 65: Queue query**
Do: select JumpLogs where initialAnalysisTimestamp IS NULL ordered newest first
Done when: logs number pending.

- **Task 66: Parser interface**
Do: a function parseLog(raw) returning time series for altitude, vspeed, gps (mock)
Done when: unit test returns arrays from mock bytes.

- **Task 67: Exit detection**
Do: detect first sustained >2000 fpm for ≥1s; store exitOffsetSec
Done when: unit test with synthetic data passes.

- **Task 68: Deployment & activation detection**
Do: 0.25g decel for 0.1s; and first RoD <2000 fpm → offsets
Done when: unit tests pass.

- **Task 69: Landing detection**
Do: RoD <100 fpm for 10s → landingOffsetSec
Done when: unit test passes.

- **Task 70: Exit timestamp & location**
Do: compute exitTimestampUTC + exitLat/Lon if GPS available
Done when: DB rows updated.

- **Task 71: Freefall metrics**
Do: compute freefall time, average fall rate (mph)
Done when: fields present in GET /api/jumps/[id].

- **Task 72: Set analysis complete**
Do: write initialAnalysisTimestamp=now(); initialAnalysisMessage on anomalies
Done when: job no longer appears in queue.

- **Task 73: Formation grouping**
Do: find logs with start times within ±120s; upsert Formation + participants
Done when: /review/fs/[id] can fetch a formation with ≥2 participants.

- **Task 74: Respect per-log visibility**
Do: store participant isVisibleToOthers=true default; update on toggle
Done when: hidden jumper not shown to others in formation API.

---

## Phase 10 — UI: Home Panels & Jump Details

- **Task 75: Right panel: My Jumps**
Do: list last 5 with date, freefall time, avg fall rate
Done when: renders from API.

- **Task 76: Right panel: Formation Jumps**
Do: list recent formations involving user
Done when: clicking opens /review/fs/[id].

- **Task 77: Center: Jump summary**
Do: when a jump selected, show exit time, deploy alt, freefall, avg fall rate
Done when: selection updates panel.

- **Task 78: Jump detail chart (Recharts)**
Do: altitude vs time line, markers for exit/deploy/landing
Done when: renders sample series.

- **Task 79: Visibility toggle UI**
Do: switch in summary card writes visibleToConnections
Done when: toggle updates DB and affects other viewer access.

- **Task 80: Notes editor**
Do: markdown textarea; save via PATCH
Done when: note persists and renders as markdown.

---

## Phase 11 — Formation Review (D3) MVP

- **Task 81: Data API**
Do: /api/formations/[id] returns participants with time-series (mock initially)
Done when: endpoint responds shape for viz.

- **Task 82: Projection math**
Do: helper to convert lat/lon/alt to local XY, choose base, compute formation frame (+X along line of flight, +Z up)
Done when: unit test of transform yields expected vectors.

- **Task 83: Canvas render loop**
Do: draw dots for participants at time t; play/pause/seek
Done when: play button animates points.

- **Task 84: Preset views**
Do: god’s-eye and -X side view toggles
Done when: buttons switch projection.

- **Task 85: Base info panel**
Do: show live fall rate (mph), normalized fall rate, AGL
Done when: values update during playback.

- **Task 86: Jumper list panel**
Do: show distance to base (ft), closure rate (fps & mph) per jumper
Done when: values update during playback.

(You can keep trajectories mocked first, then wire real series in 90–92.)

---

## Phase 12 — Lending & Proxy Users

- **Task 87: Create proxy API**
Do: /api/lending/proxy creates User{isProxy=true, proxyCreatorId}
Done when: returns proxy user id and slug.

- **Task 88: Lend device API**
Do: /api/lending/lend (owner only) sets device.lentTo + policy
Done when: DB updates reflect lending status.

- **Task 89: Reclaim device API**
Do: /api/lending/reclaim resets lentTo; reassign to owner
Done when: device state back to owner in DB.

- **Task 90: Lend UI**
Do: form to choose device, select existing user or “Create proxy…”, pick duration
Done when: successful lend shows confirmation.

- **Task 91: Auto-return (one-jump)**
Do: in scanner, after successful upload on lent device with one-jump policy, auto-reclaim (DB + assign user on device later in Phase 93)
Done when: next scan shows device back to owner (DB).

- **Task 92: Claim invitation create**
Do: /api/invitations/create for a given proxy; store expiry=30 days
Done when: returns invitation id + claim URL.

- **Task 93: Claim QR display**
Do: UI to show QR for claim URL
Done when: QR scans to correct route.

- **Task 94: Accept invitation flow**
Do: /accept-invitation/[id] with form (email, password); flips isProxy=false
Done when: user can log in; proxy retains jump history.

---

## Phase 13 — Bluetooth Device Admin Actions (Real)

Many of the capabilities described here will require UI updates to the "Devices (Admin)" page.  Specifically most new function will require adding menu menu items off a new "vertical kebab" menu button to be added to each device row displayed on the page.

The tasks described in this phase require bluetooth interactions with devices.  These commands will be initiated by the user from the web browser UI. These are all in addition to the routine Bluetooth scanning handled by the `worker/bluetooth-scanner` module.  To reduce the probability of concurrency issues, we will leave the bluetooth-scanner exclusively responsible for communicating with  devices.  The commands described below will be relayed via a Postgres DeviceCommand queue (a table). This table will specify the target device and include columns to hold any data to be sent. `id`, `sendingUser`, `createdAt`, `completedAt`, and `commandStatus` (QUEUED, SENDING, COMPLETED, DEVICE_ERROR, DEVICE_TIMEOUT).  The Bluetooth-scanner worker will now scan this queue as part of its normal work loop, transmit commands, gather responses, and update the queue entries as needed.

We expect these specific interaction commands will need to be implemented in the queue data model and later in the UI: PING (testing connection only: `smpmgr --ble MyBLEDevice os echo hello`), BLINK_ON (specifying RGB color), BLINK_OFF, ASSIGN (with `uinfo.json` file contents [ <256 bytes ]), UNPROVISION (unassign device), INITIALIZE (specifying permanent device name and PCB version).

API routes for these commands shall follow the pattern: `/api/devices/[id]/commands/<command-name>` - all command APIs will require ADMIN or SUPER_ADMIN role to invoke, except for `/led-on` and `/led-off` - there, the user must be the assignee/owner of the device.  These APIs will return the assigned `id` of the newly created command queue entry.  Progress of the command can be tracked via a new `/api/devices/[id]/commands` API, which will return a list of all commands in the queue corresponding to the specified device (only available to device assignees/owners and admins). Queue entries are deleted by sending a HTTP DELETE to `/api/devices/[id]/commands/[id]`. Entries can only be deleted after entering the DEVICE_ERROR or DEVICE_TIMEOUT states.

- **Task95a: Add DeviceCommandQueue to data model**
Do: Add the DeviceCommandQueue table to the data model. Include `targetDevice`, and the other fields described above. Add skeleton elements in worker/bluetooth-scanner to detect newly QUEUED queue entries and serial dispatch those to the target Device while updating the command's status as required.
Done: DeviceCommandQueue table exists and polled by bluetooth-scanner.ts

- **Task 95b: Bluetooth service module**
Do: production wrapper for smpmgr (exec, parse, timeout/retry) [likely already implemented in bluetooth-scanner and service module code]
Done when: file upload ('/lfs/logs/20250901/100/flight.txt'), tempo session-list (verifying that log is on the session list), file download work with device.

- **Task 96: Blink command**
Do: implement device blink via `smpmgr` tempo command set [already implemented in `bluetooth.service.ts`]; /commands API and end-to-end flow through bluetooth-scanner needs implementation.
Done when: device LED blinks on admin action (`tempo led-on red`, `tempo led-off`)

- **Task 97: Assign to user (on device)**
Do: create `/api/devices/[id]/commands/assign` API. Create "Assign to User" menu function in Device Admin list. Write `/uinfo.json` (Owner Name and Owner-slug)
Done when: device reads back updated info.

- **Task 98: Un-provision (on device)**
Do: Clear assignment by removing '/uinfo.json' file; update /api/devices/[id]/assign API to allow for nulling ownerId (when device isn't lent); implementation of end-to-end flow.
Done when: device advertised in un-provisioned state.

- **Task 99: Initialize device (with new unique name)**
Do: (candidates for initialization will advertise their name as "Tempo-BT";  there may be multiple such devices on line, each with a pseudo-random BT address); Create "Initialize" menu function in Device Admin list.; generate/set Bluetooth ID & name “Tempo-BT-xxxx” (last assigned 4-digit id should be maintained in Postgres "app configuration settings" table, seed value "0010"); to configure the device, these values should be written to NVM via settings API on device - this device functionality hasn't been implemented yet.
Done when: scanner sees new name; DB updated.

- **Task 100: Concurrency lock**
- Do: per-device mutex so scanner and admin actions don’t collide [ this is likely unneeded with queue-based command processing, as we will implement ]
- Done when: parallel commands queue instead of error.

---

## Phase 14 — Analysis Enhancements

- **Task 101: Normalized fall rate**
- Do: implement ISA density correction; produce normalized series
- Done when: unit test: higher altitude → higher unnormalized speed but similar normalized.

- **Task 102: Average jumper band (config)**
- Do: constants 110–130 mph; expose in UI band overlay
- Done when: chart shows band; values configurable in constants.

- **Task 103: Anomaly messages**
- Do: fill initialAnalysisMessage on missing GPS/short log
- Done when: shows on jump detail when applicable.

---

## Phase 15 — Visibility & Permissions Hardening

- **Task 104: Visibility rules (API)**
- Do: enforce visibleToConnections + group/connection checks on all jump reads
- Done when: unauthorized user cannot GET hidden jumps.

- **Task 105: Connections (basic)**
- Do: endpoints to send/accept/decline connection requests; “Pending Actions” list
- Done when: two users can connect and see each other’s visible jumps.

---

## Phase 16 — Export & Deletion

- **Task 106: Export my data**
- Do: /api/export zips user’s jump JSON + raw files; stream download
- Done when: archive downloads; contains expected files.

- **Task 107: Delete jump**
- Do: owner can delete a jump; remove from formations or mark removed
- Done when: jump no longer appears; formations update.

- **Task 108: Delete proxy user**
- Do: creator can delete proxy with cascade rules (or prevent if data exists)
- Done when: action behaves per policy.

- **Task 109: Delete group (admin)**
- Do: prompt for new admin if last admin; otherwise delete
- Done when: constraints enforced.

---

## Phase 17 — Polling & Performance

- **Task 110: Global polling hook**
- Do: reusable usePolling(fetcher, interval) hook
- Done when: devices & jump lists share the hook.

- **Task 111: Indexing**
- Do: DB indexes on JumpLog(userId, createdAt), Device(lastSeen), FormationParticipant(formationId)
- Done when: slow queries improved (verify via EXPLAIN).

- **Task 112: Binary size check**
- Do: reject logs > 16MB with clear message in worker
- Done when: oversized file logged & skipped gracefully.

---

## Phase 18 — Packaging & Pi Deployment

- **Task 113: Env templates**
- Do: .env.example with all keys (DB, JWT_SECRET, WORKER_TOKENS, DISCOVERY_WINDOW)
- Done when: developer can copy and run locally.

- **Task 114: Systemd services**
- Do: unit files for web, worker-bt, worker-analysis
- Done when: systemctl starts all; restart on failure.

- **Task 115: smpmgr install script**
- Do: shell script to install dependencies on Ubuntu 24.04 (BlueZ, smpmgr)
- Done when: script exits 0 and smpmgr -h works.

- **Task 116: Health endpoints**
- Do: /api/health returns OK; workers log heartbeats
- Done when: simple checks pass.

---

Each phase breaks down into atomic tasks with clear “Done when” checks.  
