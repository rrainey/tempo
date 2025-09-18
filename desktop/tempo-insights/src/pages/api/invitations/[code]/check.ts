// pages/api/invitations/[code]/check.ts
import { NextApiRequest, NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';

const prisma = new PrismaClient();

export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  const { code } = req.query;

  if (typeof code !== 'string') {
    return res.status(400).json({ error: 'Invalid invitation code' });
  }

  try {
    // Find the invitation
    const invitation = await prisma.userInvitation.findUnique({
      where: { code },
      include: {
        invitedBy: {
          select: { name: true },
        },
      },
    });

    if (!invitation) {
      return res.status(404).json({ error: 'Invitation not found' });
    }

    // Check if expired
    if (new Date() > invitation.expiresAt) {
      return res.status(400).json({ error: 'Invitation has expired' });
    }

    // Check if already used
    if (invitation.usedAt) {
      return res.status(400).json({ error: 'Invitation has already been used' });
    }

    // Check if this is for a proxy user claim
    let proxyUser = null;
    if (invitation.email.includes('@tempo.local')) {
      proxyUser = await prisma.user.findFirst({
        where: { 
          email: invitation.email,
          isProxy: true,
        },
        select: {
          id: true,
          name: true,
          slug: true,
        },
      });
    }

    return res.status(200).json({
      invitation: {
        id: invitation.id,
        code: invitation.code,
        invitedBy: invitation.invitedBy.name,
        proxyUser,
        expiresAt: invitation.expiresAt,
      },
    });
  } catch (error) {
    console.error('Check invitation error:', error);
    return res.status(500).json({ error: 'Internal server error' });
  }
}