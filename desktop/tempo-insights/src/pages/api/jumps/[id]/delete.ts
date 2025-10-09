// pages/api/jumps/[id]/delete.ts

import { NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';
import { createClient } from '@supabase/supabase-js';
import { withAuth, AuthenticatedRequest } from '../../../../lib/auth/middleware';

const prisma = new PrismaClient();

const supabase = createClient(
  process.env.NEXT_PUBLIC_SUPABASE_URL!,
  process.env.SUPABASE_SERVICE_KEY!
);

async function handler(req: AuthenticatedRequest, res: NextApiResponse) {
  if (req.method !== 'DELETE') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  if (!req.user) {
    return res.status(401).json({ error: 'Unauthorized' });
  }

  const { id } = req.query;
  if (typeof id !== 'string') {
    return res.status(400).json({ error: 'Invalid jump ID' });
  }

  try {
    // First, verify the jump exists and belongs to the user
    const jump = await prisma.jumpLog.findUnique({
      where: { id },
      include: {
        formationParticipant: {
          include: {
            formation: {
              include: {
                participants: true
              }
            }
          }
        }
      }
    });

    if (!jump) {
      return res.status(404).json({ error: 'Jump not found' });
    }

    // Only the owner or an admin can delete
    const isOwner = jump.userId === req.user.id;
    const isAdmin = req.user.role === 'ADMIN' || req.user.role === 'SUPER_ADMIN';

    if (!isOwner && !isAdmin) {
      return res.status(403).json({ error: 'Forbidden: You can only delete your own jumps' });
    }

    // Start a transaction to delete everything atomically
    await prisma.$transaction(async (tx) => {
      // If this jump is part of a formation, remove the participant entry
      if (jump.formationParticipant) {
        const formation = jump.formationParticipant.formation;
        
        // Delete the formation participant
        await tx.formationParticipant.delete({
          where: { id: jump.formationParticipant.id }
        });

        // If this was the last participant, delete the formation
        const remainingParticipants = formation.participants.filter(
          p => p.id !== jump.formationParticipant!.id
        );

        if (remainingParticipants.length === 0) {
          await tx.formationSkydive.delete({
            where: { id: formation.id }
          });
        }
      }

      // Delete the jump log from database
      await tx.jumpLog.delete({
        where: { id }
      });
    });

    // Delete the file from Supabase Storage (non-critical, do after DB)
    if (jump.storagePath) {
      try {
        const { error } = await supabase.storage
          .from('jump-logs')
          .remove([jump.storagePath]);

        if (error) {
          console.error('Failed to delete file from storage:', error);
          // Don't fail the request if storage deletion fails
        }
      } catch (error) {
        console.error('Failed to delete file from storage:', error);
        // Continue - DB deletion succeeded
      }
    }

    res.status(200).json({ 
      success: true,
      message: 'Jump deleted successfully'
    });

  } catch (error) {
    console.error('Error deleting jump:', error);
    res.status(500).json({ error: 'Internal server error' });
  } finally {
    await prisma.$disconnect();
  }
}

export default withAuth(handler);