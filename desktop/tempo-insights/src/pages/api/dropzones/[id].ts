// /pages/api/dropzones/[id].ts
import type { NextApiRequest, NextApiResponse } from 'next';
import { withAuth, AuthenticatedRequest } from '../../../lib/auth/middleware';
import { PrismaClient } from '@prisma/client';
const prisma = new PrismaClient();

export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  const { id } = req.query;

  if (typeof id !== 'string') {
    return res.status(400).json({ error: 'Invalid dropzone ID' });
  }

  if (req.method === 'GET') {
    // Get dropzone by slug (public)
    try {
      const dropzone = await prisma.dropzone.findFirst({
        where: {
          OR: [
            { id },
            { slug: id }
          ]
        },
        include: {
          _count: {
            select: { formations: true }
          }
        }
      });

      if (!dropzone) {
        return res.status(404).json({ error: 'Dropzone not found' });
      }

      return res.status(200).json({ dropzone });
    } catch (error) {
      console.error('Error fetching dropzone:', error);
      return res.status(500).json({ error: 'Failed to fetch dropzone' });
    }
  }

  else if (req.method === 'PATCH') {
    // Update dropzone - admin only
    return withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
      const user = req.user;

      if (user && user.role !== 'ADMIN' && user.role !== 'SUPER_ADMIN') {
        return res.status(403).json({ error: 'Admin access required' });
      }

      const updates: any = {};
      const { name, icaoCode, latitude, longitude, elevation, timezone, notes, isActive } = req.body;

      if (name !== undefined) updates.name = name;
      if (icaoCode !== undefined) updates.icaoCode = icaoCode;
      if (latitude !== undefined) updates.latitude = parseFloat(latitude);
      if (longitude !== undefined) updates.longitude = parseFloat(longitude);
      if (elevation !== undefined) updates.elevation = parseFloat(elevation);
      if (timezone !== undefined) updates.timezone = timezone;
      if (notes !== undefined) updates.notes = notes;
      if (isActive !== undefined) updates.isActive = isActive;

      // If name changed, update slug
      if (name !== undefined) {
        const baseSlug = name.toLowerCase()
          .replace(/[^a-z0-9]+/g, '-')
          .replace(/^-+|-+$/g, '');
        
        let slug = baseSlug;
        let counter = 1;
        while (true) {
          const existing = await prisma.dropzone.findUnique({ where: { slug } });
          if (!existing || existing.id === id) break;
          slug = `${baseSlug}-${counter}`;
          counter++;
        }
        updates.slug = slug;
      }

      try {
        const dropzone = await prisma.dropzone.update({
          where: { id },
          data: updates
        });

        return res.status(200).json({ dropzone });
      } catch (error) {
        console.error('Error updating dropzone:', error);
        return res.status(500).json({ error: 'Failed to update dropzone' });
      }
    })(req, res);
  }

  else if (req.method === 'DELETE') {
    // Delete dropzone - admin only
    return withAuth(async (req: AuthenticatedRequest, res: NextApiResponse) => {
      const user = req.user;

      if (user && user.role !== 'ADMIN' && user.role !== 'SUPER_ADMIN') {
        return res.status(403).json({ error: 'Admin access required' });
      }

      try {
        // Check if dropzone has formations
        const dropzone = await prisma.dropzone.findUnique({
          where: { id },
          include: {
            _count: {
              select: { formations: true }
            }
          }
        });

        if (!dropzone) {
          return res.status(404).json({ error: 'Dropzone not found' });
        }

        if (dropzone._count.formations > 0) {
          return res.status(400).json({ 
            error: 'Cannot delete dropzone with associated formations' 
          });
        }

        await prisma.dropzone.delete({
          where: { id }
        });

        return res.status(200).json({ message: 'Dropzone deleted successfully' });
      } catch (error) {
        console.error('Error deleting dropzone:', error);
        return res.status(500).json({ error: 'Failed to delete dropzone' });
      }
    })(req, res);
  }

  return res.status(405).json({ error: 'Method not allowed' });
}