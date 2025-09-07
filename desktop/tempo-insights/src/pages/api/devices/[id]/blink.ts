import { requireAdmin, AuthenticatedRequest } from '../../../../lib/auth/middleware';
import { NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';

const prisma = new PrismaClient();

export default requireAdmin(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  const { id } = req.query;

  if (typeof id !== 'string') {
    return res.status(400).json({ error: 'Invalid device ID' });
  }

  try {
    // Get the device
    const device = await prisma.device.findUnique({
      where: { id },
    });

    if (!device) {
      return res.status(404).json({ error: 'Device not found' });
    }

    // TODO: In Phase 8, this will send actual Bluetooth command
    // For now, just log the intent
    console.log(`[BLINK] Sending blink command to device ${device.name} (${device.bluetoothId})`);

    // Update lastSeen to simulate device communication
    await prisma.device.update({
      where: { id },
      data: {
        lastSeen: new Date(),
      },
    });

    return res.status(200).json({
      success: true,
      message: `Blink command sent to ${device.name}`,
      device: {
        id: device.id,
        name: device.name,
        bluetoothId: device.bluetoothId,
      },
    });
  } catch (error) {
    console.error('Blink device error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});
