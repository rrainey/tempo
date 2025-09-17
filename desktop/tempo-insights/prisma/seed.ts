import { PrismaClient, DeviceState } from '@prisma/client';
import bcrypt from 'bcrypt';
import { config } from 'dotenv';

// Load environment variables from .env.local
config({ path: '.env.local' });

const prisma = new PrismaClient();

async function main() {
  // Create admin user
  const hashedPassword = await bcrypt.hash('admin123', 10);

  const admin = await prisma.user.upsert({
    where: { email: 'admin@tempoinsights.local' },
    update: {},
    create: {
      email: 'admin@tempoinsights.local',
      password: hashedPassword,
      name: 'Admin User',
      slug: 'admin',
      role: 'SUPER_ADMIN',
    },
  });

  console.log({ admin });

  // Create test devices
  const device1 = await prisma.device.upsert({
    where: { bluetoothId: 'AA:BB:CC:DD:EE:01' },
    update: {},
    create: {
      bluetoothId: 'AA:BB:CC:DD:EE:01',
      name: 'Tempo Device 01',
      state: DeviceState.ACTIVE,
      ownerId: admin.id,
      lastSeen: new Date(),
    },
  });

  const device2 = await prisma.device.upsert({
    where: { bluetoothId: 'AA:BB:CC:DD:EE:02' },
    update: {},
    create: {
      bluetoothId: 'AA:BB:CC:DD:EE:02',
      name: 'Tempo Device 02',
      state: DeviceState.ACTIVE,
      ownerId: admin.id,
      lastSeen: new Date(Date.now() - 10 * 60 * 1000), // 10 minutes ago
    },
  });

  const device3 = await prisma.device.upsert({
    where: { bluetoothId: 'AA:BB:CC:DD:EE:03' },
    update: {},
    create: {
      bluetoothId: 'AA:BB:CC:DD:EE:03',
      name: 'Tempo Device 03',
      state: DeviceState.PROVISIONING,
      ownerId: admin.id,
      lastSeen: null,
    },
  });

  const dropzone1 = await prisma.dropzone.upsert({
    where: { slug: 'skydive-elsinore' },
    update: {},
    create: {
      name: "Skydive Elsinore",
      slug: "skydive-elsinore", 
      icaoCode: "F14",
      latitude: 33.6320,
      longitude: -117.2510,
      elevation: 436.5, // meters MSL
      timezone: "America/Los_Angeles",
      notes: "Lake Elsinore, CA"
    },
  });

  const dropzone2 = await prisma.dropzone.upsert({
    where: { slug: 'ssd-d-area' },
    update: {},
    create: {
      name: "Spaceland Dallas",
      slug: "ssd-d-area",
      icaoCode: "48TX",
      latitude: 33.450034, 
      longitude: -96.378904,
      elevation: 241, // meters MSL
      timezone: "America/Chicago",
      notes: "Whitewright, TX"
    },
  });

  const dropzone3 = await prisma.dropzone.upsert({
    where: { slug: 'ssd-c-area' },
    update: {},
    create: {
      name: "Spaceland Dallas",
      slug: "ssd-c-area",
      icaoCode: "48TX",
      latitude: 33.451155, 
      longitude: -96.378161, 
      elevation: 241, // meters MSL
      timezone: "America/Chicago",
      notes: "Whitewright, TX"
    },
  });

  console.log({ device1, device2, device3, dropzone1, dropzone2, dropzone3 });
}

main()
  .catch((e) => {
    console.error(e);
    process.exit(1);
  })
  .finally(async () => {
    await prisma.$disconnect();
  });
