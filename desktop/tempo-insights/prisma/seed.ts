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

  console.log({ device1, device2, device3 });
}

main()
  .catch((e) => {
    console.error(e);
    process.exit(1);
  })
  .finally(async () => {
    await prisma.$disconnect();
  });
