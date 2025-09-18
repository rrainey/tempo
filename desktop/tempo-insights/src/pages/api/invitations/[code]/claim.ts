// pages/api/invitations/[code]/claim.ts
import { NextApiRequest, NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';
import bcrypt from 'bcrypt';

const prisma = new PrismaClient();

export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  const { code } = req.query;
  const { email, password } = req.body;

  if (typeof code !== 'string') {
    return res.status(400).json({ error: 'Invalid invitation code' });
  }

  if (!email || !password || typeof email !== 'string' || typeof password !== 'string') {
    return res.status(400).json({ error: 'Email and password are required' });
  }

  if (password.length < 8) {
    return res.status(400).json({ error: 'Password must be at least 8 characters' });
  }

  try {
    // Start a transaction
    const result = await prisma.$transaction(async (tx) => {
      // Find and validate invitation
      const invitation = await tx.userInvitation.findUnique({
        where: { code },
        include: {
          invitedBy: true,
        },
      });

      if (!invitation) {
        throw new Error('Invitation not found');
      }

      if (new Date() > invitation.expiresAt) {
        throw new Error('Invitation has expired');
      }

      if (invitation.usedAt) {
        throw new Error('Invitation has already been used');
      }

      // Check if this is for a proxy user
      let user;
      if (invitation.email.includes('@tempo.local')) {
        // This is a proxy user claim
        user = await tx.user.findFirst({
          where: { 
            email: invitation.email,
            isProxy: true,
          },
        });

        if (!user) {
          throw new Error('Proxy user not found');
        }

        // Hash the password
        const hashedPassword = await bcrypt.hash(password, 10);

        // Update proxy user to full user
        user = await tx.user.update({
          where: { id: user.id },
          data: {
            email,
            password: hashedPassword,
            isProxy: false,
            // Keep the proxyCreatorId for reference
          },
        });

        console.log(`[CLAIM] Proxy user ${user.name} (${user.id}) claimed account with email ${email}`);
      } else {
        // This would be a regular invitation (not implemented in this flow)
        throw new Error('Regular invitations not supported in this endpoint');
      }

      // Mark invitation as used
      await tx.userInvitation.update({
        where: { id: invitation.id },
        data: {
          usedAt: new Date(),
          usedById: user.id,
        },
      });

      return { user };
    });

    return res.status(200).json({
      success: true,
      message: 'Account successfully claimed',
      user: {
        id: result.user.id,
        name: result.user.name,
        email: result.user.email,
        slug: result.user.slug,
      },
    });
  } catch (error) {
    console.error('Claim invitation error:', error);
    
    if (error instanceof Error) {
      return res.status(400).json({ error: error.message });
    }
    
    return res.status(500).json({ error: 'Internal server error' });
  }
}