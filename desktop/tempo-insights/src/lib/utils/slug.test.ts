import { createBaseSlug } from './slug';

describe('Slug generator', () => {
  describe('createBaseSlug', () => {
    it('should convert name to slug', () => {
      expect(createBaseSlug('Bill Jones')).toBe('bill-jones');
      expect(createBaseSlug('John O\'Brien')).toBe('john-o-brien');
      expect(createBaseSlug('  Mary Ann  ')).toBe('mary-ann');
      expect(createBaseSlug('José García')).toBe('jos-garc-a');
      expect(createBaseSlug('Test@User#123')).toBe('test-user-123');
    });

    it('should handle edge cases', () => {
      expect(createBaseSlug('')).toBe('');
      expect(createBaseSlug('---')).toBe('');
      expect(createBaseSlug('a')).toBe('a');
      expect(createBaseSlug('A B C')).toBe('a-b-c');
    });

    it('should collapse multiple hyphens', () => {
      expect(createBaseSlug('test   user')).toBe('test-user');
      expect(createBaseSlug('test---user')).toBe('test-user');
    });
  });

  // Note: Testing the async functions would require mocking Prisma
  // For now, we test the base slug creation logic
});
