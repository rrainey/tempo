// src/pages/api/connections/[id]/index.ts
import type { NextApiResponse } from 'next';
import { withAuth, AuthenticatedRequest } from '../../../../lib/auth/middleware';
import { PrismaClient } from '@prisma/client';

const prisma = new PrismaClient();

export default withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
  const { id: requestId } = req.query;
  
  if (!requestId || typeof requestId !== 'string') {
    return res.status(400).json({ error: 'Invalid request ID' });
  }

  if (req.method === 'DELETE') {
    try {
      const userId = req.user!.id;

      // Get the connection request
      const request = await prisma.connectionRequest.findUnique({
        where: { id: requestId }
      });

      if (!request) {
        return res.status(404).json({ error: 'Connection request not found' });
      }

      // Verify the current user is the sender
      if (request.fromUserId !== userId) {
        return res.status(403).json({ error: 'You can only cancel your own requests' });
      }

      // Check if already responded
      if (request.status !== 'PENDING') {
        return res.status(400).json({ 
          error: `Request already ${request.status.toLowerCase()}` 
        });
      }

      // Delete the request
      await prisma.connectionRequest.delete({
        where: { id: requestId }
      });

      return res.status(200).json({ message: 'Connection request cancelled' });

    } catch (error) {
      console.error('Cancel connection request error:', error);
      return res.status(500).json({ error: 'Internal server error' });
    }
  } else {
    return res.status(405).json({ error: 'Method not allowed' });
  }
});