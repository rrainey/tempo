import { withAuth, AuthenticatedRequest } from '../../../../lib/auth/middleware';
import { NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';
import { z } from 'zod';

const prisma = new PrismaClient();

const visibilitySchema = z.object({
  visibleToConnections: z.boolean(),
});

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  if (req.method !== 'PATCH') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  const { id } = req.query;

  if (typeof id !== 'string') {
    return res.status(400).json({ error: 'Invalid jump ID' });
  }

  try {
    const { visibleToConnections } = visibilitySchema.parse(req.body);

    // Get the jump and verify ownership
    const jump = await prisma.jumpLog.findUnique({
      where: { id },
    });

    if (!jump) {
      return res.status(404).json({ error: 'Jump not found' });
    }

    if (jump.userId !== req.user!.id) {
      return res.status(403).json({ error: 'You can only modify your own jumps' });
    }

    // Update visibility
    const updatedJump = await prisma.jumpLog.update({
      where: { id },
      data: { visibleToConnections },
    });

    return res.status(200).json({
      success: true,
      visibleToConnections: updatedJump.visibleToConnections,
      message: `Jump visibility ${visibleToConnections ? 'enabled' : 'disabled'}`,
    });
  } catch (error) {
    if (error instanceof z.ZodError) {
      return res.status(400).json({
        error: 'Invalid request data',
        details: error.issues,
      });
    }

    console.error('Update jump visibility error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});
