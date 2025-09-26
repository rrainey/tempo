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