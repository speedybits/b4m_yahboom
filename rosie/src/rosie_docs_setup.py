#!/usr/bin/env python3
"""
ROSIE Docs Setup Wizard
Interactive setup for Google Docs/Drive integration with ROSIE

This script:
1. Guides you through Google Drive API authentication
2. Automatically finds Rosie_Knowledge/Public and Rosie_Knowledge/Private folders
3. Creates docs_config.json with the found folder IDs

Expected folder structure in Google Drive:
  Rosie_Knowledge/
    ├── Public/   (always synced to RAG)
    └── Private/  (synced only when private mode enabled)
"""

import os
import json
from pathlib import Path
from google.auth.transport.requests import Request
from google.oauth2.credentials import Credentials
from google_auth_oauthlib.flow import InstalledAppFlow
from googleapiclient.discovery import build
from googleapiclient.errors import HttpError

# Scopes for read-only access to Drive
SCOPES = ['https://www.googleapis.com/auth/drive.readonly']

# File paths (relative to rosie root directory - parent of src/)
ROSIE_DIR = Path(__file__).parent.parent
CONFIG_FILE = ROSIE_DIR / 'data' / 'docs_config.json'


def get_client_config():
    """
    Build OAuth client configuration from environment variables

    Required environment variables in ~/.bashrc:
        GOOGLE_DOCS_CLIENT_ID (or falls back to GOOGLE_CALENDAR_CLIENT_ID)
        GOOGLE_DOCS_CLIENT_SECRET (or falls back to GOOGLE_CALENDAR_CLIENT_SECRET)

    Returns:
        Dictionary with OAuth client configuration
    """
    # Check for docs-specific credentials first, fall back to calendar credentials
    client_id = os.getenv('GOOGLE_DOCS_CLIENT_ID') or os.getenv('GOOGLE_CALENDAR_CLIENT_ID')
    client_secret = os.getenv('GOOGLE_DOCS_CLIENT_SECRET') or os.getenv('GOOGLE_CALENDAR_CLIENT_SECRET')

    if not client_id or not client_secret:
        print("\n[ERROR] Google Docs/Drive credentials not found in environment variables!")
        print("\nPlease add these to your ~/.bashrc:")
        print("  export GOOGLE_DOCS_CLIENT_ID='your-client-id-here'")
        print("  export GOOGLE_DOCS_CLIENT_SECRET='your-client-secret-here'")
        print("\nOR reuse your Google Calendar credentials (same OAuth app):")
        print("  export GOOGLE_CALENDAR_CLIENT_ID='your-client-id-here'")
        print("  export GOOGLE_CALENDAR_CLIENT_SECRET='your-client-secret-here'")
        print("\nTo get these credentials:")
        print("1. Go to https://console.cloud.google.com/")
        print("2. Create a new project (or select existing)")
        print("3. Enable Google Drive API")
        print("4. Create OAuth 2.0 Desktop credentials")
        print("5. Copy the Client ID and Client Secret")
        print("6. Add them to ~/.bashrc as shown above")
        print("7. Run: source ~/.bashrc")
        return None

    # Build OAuth client config dictionary
    # This matches the structure of credentials.json for "installed" app type
    return {
        "installed": {
            "client_id": client_id,
            "client_secret": client_secret,
            "auth_uri": "https://accounts.google.com/o/oauth2/auth",
            "token_uri": "https://oauth2.googleapis.com/token",
            "auth_provider_x509_cert_url": "https://www.googleapis.com/oauth2/v1/certs",
            "redirect_uris": ["http://localhost"]
        }
    }


def get_token_data():
    """
    Get stored token data from environment variable

    Required environment variable in ~/.bashrc:
        GOOGLE_DOCS_TOKEN

    Returns:
        Dictionary with token data, or None if not found
    """
    token_json = os.getenv('GOOGLE_DOCS_TOKEN')
    if token_json:
        try:
            return json.loads(token_json)
        except json.JSONDecodeError:
            print("[AUTH] Warning: Invalid token data in GOOGLE_DOCS_TOKEN")

    return None


def save_token_data(creds):
    """
    Display token for user to save to ~/.bashrc environment variable

    IMPORTANT: Tokens are ONLY stored in environment variables, never in files.
    This ensures secrets are kept secure in ~/.bashrc.

    Args:
        creds: Credentials object with token data
    """
    token_json = creds.to_json()

    # Set for current session
    os.environ['GOOGLE_DOCS_TOKEN'] = token_json

    # Print instructions for ~/.bashrc setup (REQUIRED, not optional)
    print("\n" + "=" * 80)
    print("IMPORTANT: ADD TOKEN TO ~/.bashrc")
    print("=" * 80)
    print("\nAdd the following line to your ~/.bashrc file:")
    print("\n" + "-" * 80)
    print(f"export GOOGLE_DOCS_TOKEN='{token_json}'")
    print("-" * 80)
    print("\nThen run: source ~/.bashrc")
    print("\nIMPORTANT: Keep this token secret - treat it like a password!")
    print("The token is set for this session, but you MUST add it to ~/.bashrc")
    print("for ROSIE to work after restarting your terminal.")
    print("=" * 80)


def authenticate():
    """
    Authenticate with Google Drive API using OAuth2 with environment variables

    Returns:
        Credentials object for API access
    """
    creds = None

    # Get OAuth client config from environment variables
    client_config = get_client_config()
    if not client_config:
        return None

    # Check if we have existing token from environment variable
    token_data = get_token_data()
    if token_data:
        print("[AUTH] Loading existing credentials from GOOGLE_DOCS_TOKEN")
        creds = Credentials.from_authorized_user_info(token_data, SCOPES)

    # If credentials are missing or invalid, run OAuth flow
    if not creds or not creds.valid:
        if creds and creds.expired and creds.refresh_token:
            print("[AUTH] Refreshing expired credentials...")
            try:
                creds.refresh(Request())
                save_token_data(creds)
            except Exception as e:
                # Refresh failed (e.g., invalid_grant) - need full re-auth
                print(f"[AUTH] Refresh failed: {e}")
                print("[AUTH] Token expired or revoked - starting fresh authentication...")
                print("\n[AUTH] Starting OAuth2 authentication flow...")
                print("[AUTH] A browser window will open for you to authorize access")
                flow = InstalledAppFlow.from_client_config(client_config, SCOPES)
                creds = flow.run_local_server(port=0)
                save_token_data(creds)
        else:
            print("\n[AUTH] Starting OAuth2 authentication flow...")
            print("[AUTH] A browser window will open for you to authorize access")
            flow = InstalledAppFlow.from_client_config(client_config, SCOPES)
            creds = flow.run_local_server(port=0)
            save_token_data(creds)

    print("[AUTH] ✓ Authentication successful!")
    return creds


def find_rosie_folders(service):
    """
    Automatically find Rosie_Knowledge/Public and Rosie_Knowledge/Private folders

    Args:
        service: Google Drive API service object

    Returns:
        Dictionary with 'public' and 'private' folder IDs, or None if not found
        Format: {'public': {'id': '...', 'name': 'Rosie_Knowledge/Public'},
                 'private': {'id': '...', 'name': 'Rosie_Knowledge/Private'}}
    """
    try:
        print("\n[DRIVE] Looking for Rosie_Knowledge folder structure...")

        # Step 1: Find Rosie_Knowledge parent folder
        results = service.files().list(
            q="name='Rosie_Knowledge' and mimeType='application/vnd.google-apps.folder' and trashed=false",
            fields="files(id, name)"
        ).execute()

        parent_folders = results.get('files', [])

        if not parent_folders:
            print("[DRIVE] Rosie_Knowledge folder not found")
            return None

        if len(parent_folders) > 1:
            print(f"[DRIVE] Warning: Found {len(parent_folders)} 'Rosie_Knowledge' folders, using first one")

        parent_id = parent_folders[0]['id']
        print(f"[DRIVE] Found Rosie_Knowledge folder (ID: {parent_id})")

        # Step 2: Find Public subfolder
        results = service.files().list(
            q=f"name='Public' and '{parent_id}' in parents and mimeType='application/vnd.google-apps.folder' and trashed=false",
            fields="files(id, name)"
        ).execute()

        public_folders = results.get('files', [])

        # Step 3: Find Private subfolder
        results = service.files().list(
            q=f"name='Private' and '{parent_id}' in parents and mimeType='application/vnd.google-apps.folder' and trashed=false",
            fields="files(id, name)"
        ).execute()

        private_folders = results.get('files', [])

        # Build result dictionary
        found_folders = {}

        if public_folders:
            found_folders['public'] = {
                'id': public_folders[0]['id'],
                'name': 'Rosie_Knowledge/Public'
            }
            print(f"[DRIVE] Found Public subfolder (ID: {public_folders[0]['id']})")
        else:
            print("[DRIVE] Public subfolder not found")

        if private_folders:
            found_folders['private'] = {
                'id': private_folders[0]['id'],
                'name': 'Rosie_Knowledge/Private'
            }
            print(f"[DRIVE] Found Private subfolder (ID: {private_folders[0]['id']})")
        else:
            print("[DRIVE] Private subfolder not found")

        return found_folders if found_folders else None

    except HttpError as error:
        print(f"[ERROR] Failed to search for folders: {error}")
        return None


def save_config(public_folder, private_folder):
    """
    Save found folders to configuration file

    Args:
        public_folder: Dictionary with 'id' and 'name' for public folder (or None)
        private_folder: Dictionary with 'id' and 'name' for private folder (or None)
    """
    config = {
        "folders": {},
        "sync_settings": {
            "file_types": ["application/vnd.google-apps.document", "application/pdf"]
        }
    }

    # Add public folder if found
    if public_folder:
        config["folders"]["public"] = {
            "name": public_folder['name'],
            "id": public_folder['id'],
            "enabled": True
        }

    # Add private folder if found
    if private_folder:
        config["folders"]["private"] = {
            "name": private_folder['name'],
            "id": private_folder['id'],
            "enabled": True
        }

    CONFIG_FILE.parent.mkdir(parents=True, exist_ok=True)
    with open(CONFIG_FILE, 'w') as f:
        json.dump(config, f, indent=2)

    print(f"\n[CONFIG] Configuration saved to {CONFIG_FILE}")


def main():
    """Main setup wizard flow"""
    print("=" * 80)
    print("ROSIE GOOGLE DOCS/DRIVE SETUP WIZARD")
    print("=" * 80)
    print("\nThis wizard will help you connect ROSIE to your Google Drive.")
    print("\nROSIE expects the following folder structure in your Google Drive:")
    print("  Rosie_Knowledge/")
    print("    ├── Public/   (always synced to RAG)")
    print("    └── Private/  (synced only when private mode enabled)")

    # Step 1: Authenticate
    print("\n" + "-" * 80)
    print("STEP 1: Authentication")
    print("-" * 80)

    creds = authenticate()
    if not creds:
        return

    # Build Drive API service
    try:
        service = build('drive', 'v3', credentials=creds)
    except Exception as e:
        print(f"[ERROR] Failed to build Drive service: {e}")
        return

    # Step 2: Find Rosie folders
    print("\n" + "-" * 80)
    print("STEP 2: Finding Rosie_Knowledge Folders")
    print("-" * 80)

    found_folders = find_rosie_folders(service)

    if not found_folders:
        print("\n[ERROR] Required folder structure not found!")
        print("\nPlease create the following folders in your Google Drive:")
        print("  1. Create a folder named 'Rosie_Knowledge'")
        print("  2. Inside Rosie_Knowledge, create two subfolders:")
        print("     - Public  (for documents always accessible to ROSIE)")
        print("     - Private (for documents only accessible in private mode)")
        print("\nAfter creating these folders, run this setup wizard again.")
        return

    # Step 3: Save configuration
    print("\n" + "-" * 80)
    print("STEP 3: Save Configuration")
    print("-" * 80)

    public_folder = found_folders.get('public')
    private_folder = found_folders.get('private')

    save_config(public_folder, private_folder)

    # Step 4: Summary and next steps
    print("\n" + "=" * 80)
    print("SETUP COMPLETE!")
    print("=" * 80)

    if public_folder and private_folder:
        print("\nBoth Public and Private folders found and configured.")
    elif public_folder:
        print("\nOnly Public folder found. Private folder not configured.")
    elif private_folder:
        print("\nOnly Private folder found. Public folder not configured.")

    print("\nNext steps:")
    print(f"1. Run the sync script to fetch Google Docs:")
    print(f"   python3 {ROSIE_DIR}/src/rosie_docs_sync.py")
    print(f"\n2. The sync script will:")
    print(f"   - Fetch Google Docs from configured folders")
    print(f"   - Convert to markdown and save to knowledge_base/google_docs/")
    print(f"   - Update ROSIE's RAG knowledge base")
    print(f"\n3. Set up automatic syncing with cron (optional):")
    print(f"   0 */6 * * * cd {ROSIE_DIR} && python3 src/rosie_docs_sync.py")
    print(f"\n4. Add documents to your Google Drive folders:")
    print(f"   - Public folder: Documents always available to ROSIE")
    print(f"   - Private folder: Documents only available in private mode")
    print("\n" + "=" * 80)


if __name__ == '__main__':
    main()
