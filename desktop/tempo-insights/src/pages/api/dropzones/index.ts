// /pages/api/dropzones/index.ts
import type { NextApiRequest, NextApiResponse } from 'next';
import { PrismaClient } from '@prisma/client';
import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';

const prisma = new PrismaClient();

async function handler(req: NextApiRequest, res: NextApiResponse) {
  if (req.method === 'GET') {
    // List dropzones - public endpoint
    try {
      const dropzones = await prisma.dropzone.findMany({
        where: { isActive: true },
        orderBy: { name: 'asc' },
        select: {
          id: true,
          name: true,
          slug: true,
          icaoCode: true,
          latitude: true,
          longitude: true,
          elevation: true,
          timezone: true,
          _count: {
            select: { formations: true }
          }
        }
      });

      return res.status(200).json({ dropzones });
    } catch (error) {
      console.error('Error fetching dropzones:', error);
      return res.status(500).json({ error: 'Failed to fetch dropzones' });
    }
  } 
  
  else if (req.method === 'POST') {
    // Create dropzone - admin only
    return withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
      const user = req.user;

      if (user && user.role !== 'ADMIN' && user.role !== 'SUPER_ADMIN') {
        return res.status(403).json({ error: 'Admin access required' });
      }

      const { name, icaoCode, latitude, longitude, elevation, timezone, notes } = req.body;

      // Validate required fields
      if (!name || !latitude || !longitude || !elevation || !timezone) {
        return res.status(400).json({ error: 'Missing required fields' });
      }

      try {
        // Generate slug from name
        const baseSlug = name.toLowerCase()
          .replace(/[^a-z0-9]+/g, '-')
          .replace(/^-+|-+$/g, '');
        
        // Check for existing slugs and append number if needed
        let slug = baseSlug;
        let counter = 1;
        while (await prisma.dropzone.findUnique({ where: { slug } })) {
          slug = `${baseSlug}-${counter}`;
          counter++;
        }

        const dropzone = await prisma.dropzone.create({
          data: {
            name,
            slug,
            icaoCode,
            latitude: parseFloat(latitude),
            longitude: parseFloat(longitude),
            elevation: parseFloat(elevation),
            timezone,
            notes
          }
        });

        return res.status(201).json({ dropzone });
      } catch (error) {
        console.error('Error creating dropzone:', error);
        return res.status(500).json({ error: 'Failed to create dropzone' });
      }
    })(req, res);
  }

  return res.status(405).json({ error: 'Method not allowed' });
}

export default handler;