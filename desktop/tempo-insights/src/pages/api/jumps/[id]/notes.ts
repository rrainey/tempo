import { withAuth, AuthenticatedRequest } from '../../../../lib/auth/middleware';
import { NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';
import { z } from 'zod';

const prisma = new PrismaClient();

const notesSchema = z.object({
  notes: z.string().max(5000).nullable(),
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
    const { notes } = notesSchema.parse(req.body);

    // Get the jump and verify ownership
    const jump = await prisma.jumpLog.findUnique({
      where: { id },
    });

    if (!jump) {
      return res.status(404).json({ error: 'Jump not found' });
    }

    if (jump.userId !== req.user!.id) {
      return res.status(403).json({ error: 'You can only edit notes for your own jumps' });
    }

    // Update notes
    const updatedJump = await prisma.jumpLog.update({
      where: { id },
      data: { notes },
      select: {
        id: true,
        notes: true,
        updatedAt: true,
      },
    });

    return res.status(200).json({
      success: true,
      jump: updatedJump,
      message: 'Notes updated successfully',
    });
  } catch (error) {
    if (error instanceof z.ZodError) {
      return res.status(400).json({
        error: 'Invalid request data',
        details: error.issues,
      });
    }

    console.error('Update jump notes error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
});
