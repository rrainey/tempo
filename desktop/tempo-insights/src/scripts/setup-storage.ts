// scripts/setup-storage.ts
import { createClient } from '@supabase/supabase-js';
import { config } from 'dotenv';

// Load environment variables
config({ path: '.env' });

const supabase = createClient(
  process.env.NEXT_PUBLIC_SUPABASE_URL!,
  process.env.SUPABASE_SERVICE_KEY!
);

async function setupStorage() {
  try {
    // First check if bucket exists
    const { data: buckets, error: listError } = await supabase.storage.listBuckets();
    
    if (listError) {
      console.error('Error listing buckets:', listError);
      throw listError;
    }

    const bucketExists = buckets?.some(bucket => bucket.name === 'jump-logs');

    if (bucketExists) {
      console.log('Bucket "jump-logs" already exists');
      return;
    }

    // Create bucket if it doesn't exist
    console.log('Creating bucket "jump-logs"...');
    const { data, error } = await supabase.storage.createBucket('jump-logs', {
      public: false,
      fileSizeLimit: 16777216, // 16MB limit
      allowedMimeTypes: ['application/octet-stream', 'text/csv', 'text/plain']
    });

    if (error) {
      console.error('Error creating bucket:', error);
      throw error;
    }

    console.log('Storage bucket created successfully:', data);

    // Set up storage policies if needed
    // Note: Storage policies are managed through Supabase dashboard or SQL
    console.log('\nNext steps:');
    console.log('1. Go to Supabase dashboard -> Storage');
    console.log('2. Click on the "jump-logs" bucket');
    console.log('3. Go to Policies tab');
    console.log('4. Add appropriate RLS policies for your use case');
    console.log('\nExample policy for authenticated users to read their own files:');
    console.log(`
      -- Allow users to read their own files
      CREATE POLICY "Users can view own jump logs" 
      ON storage.objects FOR SELECT 
      USING (bucket_id = 'jump-logs' AND auth.uid()::text = (storage.foldername(name))[1]);
    `);

  } catch (error) {
    console.error('Setup failed:', error);
    throw error;
  }
}

// Run the setup
setupStorage()
  .then(() => {
    console.log('\nStorage setup completed successfully!');
    process.exit(0);
  })
  .catch((error) => {
    console.error('\nStorage setup failed:', error);
    process.exit(1);
  });