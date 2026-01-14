#!/usr/bin/env python3
"""
ROSIE Google Docs Sync Script
Syncs Google Docs and PDFs from hardcoded folder structure to ROSIE's RAG knowledge base

This script:
1. Reads docs_config.json for folder IDs
2. Authenticates with Google Drive API
3. Fetches documents from Rosie_Knowledge/Public and Rosie_Knowledge/Private folders
4. Downloads Google Docs as plain text, PDFs with text extraction
5. Saves Public to knowledge_base/google_docs/
6. Saves Private to knowledge_base/private/google_docs/
7. Cleans up docs that no longer exist in source folders
8. ROSIE's RAG system automatically indexes the updates

Run manually or via cron job (recommended: every hour)
"""

import os
import json
import re
import datetime
from pathlib import Path
from google.auth.transport.requests import Request
from google.oauth2.credentials import Credentials
from googleapiclient.discovery import build
from googleapiclient.errors import HttpError

# PDF support (optional dependency)
try:
    from pypdf import PdfReader
    import io
    from googleapiclient.http import MediaIoBaseDownload
    PDF_SUPPORT = True
except ImportError:
    PDF_SUPPORT = False

# File paths (relative to rosie root directory - parent of src/)
ROSIE_DIR = Path(__file__).parent.parent
CONFIG_FILE = ROSIE_DIR / 'data' / 'docs_config.json'
KNOWLEDGE_BASE_DIR = ROSIE_DIR / 'knowledge_base'
DOCS_OUTPUT_DIR = KNOWLEDGE_BASE_DIR / 'google_docs'
PRIVATE_DOCS_OUTPUT_DIR = KNOWLEDGE_BASE_DIR / 'private' / 'google_docs'

# Scopes for read-only Drive access
SCOPES = ['https://www.googleapis.com/auth/drive.readonly']


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
    Update token in current session and prompt user to update ~/.bashrc if refreshed

    IMPORTANT: Tokens are ONLY stored in environment variables, never in files.

    Args:
        creds: Credentials object with updated token data
    """
    token_json = creds.to_json()
    os.environ['GOOGLE_DOCS_TOKEN'] = token_json

    # Alert user that token was refreshed and needs updating in ~/.bashrc
    print("[AUTH] Token refreshed for this session")
    print("[AUTH] If sync fails after restart, update GOOGLE_DOCS_TOKEN in ~/.bashrc:")
    print(f"[AUTH] export GOOGLE_DOCS_TOKEN='{token_json}'")


def load_config():
    """
    Load Google Docs configuration from docs_config.json

    Expected structure:
    {
        "folders": {
            "public": {"name": "Rosie_Knowledge/Public", "id": "...", "enabled": true},
            "private": {"name": "Rosie_Knowledge/Private", "id": "...", "enabled": true}
        },
        "sync_settings": {...}
    }

    Returns:
        Dictionary with folder settings, or None if not found
    """
    if not CONFIG_FILE.exists():
        print(f"[ERROR] Configuration file not found: {CONFIG_FILE}")
        print("Please run rosie_docs_setup.py to configure Google Docs sync")
        return None

    with open(CONFIG_FILE, 'r') as f:
        config = json.load(f)

    folders = config.get('folders', {})
    enabled_count = sum(1 for f in folders.values() if f.get('enabled', True))
    print(f"[CONFIG] Loaded configuration with {enabled_count} enabled folder(s)")
    return config


def authenticate():
    """
    Authenticate with Google Drive API using environment variable token

    Returns:
        Credentials object, or None if authentication fails
    """
    token_data = get_token_data()
    if not token_data:
        print(f"[ERROR] GOOGLE_DOCS_TOKEN not found in environment!")
        print("Please run authentication setup first to generate token")
        return None

    creds = Credentials.from_authorized_user_info(token_data, SCOPES)

    # Refresh if expired
    if creds.expired and creds.refresh_token:
        print("[AUTH] Refreshing expired credentials...")
        creds.refresh(Request())
        save_token_data(creds)

    return creds


def sanitize_filename(name):
    """
    Sanitize document name for filesystem

    Args:
        name: Original document name

    Returns:
        Safe filename with .md extension
    """
    # Remove special chars, keep alphanumeric, spaces, dashes, underscores
    safe = re.sub(r'[^\w\s-]', '', name)
    # Replace spaces with underscores, strip, and truncate
    safe = safe.strip().replace(' ', '_')[:100]
    return safe + '.md'


def list_docs_in_folder(service, folder_id, folder_name, include_subfolders=False):
    """
    List all Google Docs and PDFs in a folder (and optionally subfolders)

    Args:
        service: Google Drive API service object
        folder_id: Folder ID to scan
        folder_name: Human-readable folder name
        include_subfolders: Whether to recursively scan subfolders

    Returns:
        List of document dictionaries with id, name, modifiedTime, mimeType, folder_name
    """
    all_docs = []

    try:
        # Query for Google Docs AND PDFs in this folder
        query = f"'{folder_id}' in parents and (mimeType='application/vnd.google-apps.document' or mimeType='application/pdf') and trashed=false"

        print(f"[SCAN] Scanning folder '{folder_name}' for Google Docs and PDFs...")

        page_token = None
        while True:
            results = service.files().list(
                q=query,
                pageSize=100,
                fields="nextPageToken, files(id, name, modifiedTime, mimeType)",
                pageToken=page_token
            ).execute()

            docs = results.get('files', [])
            for doc in docs:
                doc['folder_name'] = folder_name

            all_docs.extend(docs)

            page_token = results.get('nextPageToken')
            if not page_token:
                break

        print(f"[SCAN] Found {len(all_docs)} document(s) in '{folder_name}'")

        # Recursively scan subfolders if enabled
        if include_subfolders:
            subfolder_query = f"'{folder_id}' in parents and mimeType='application/vnd.google-apps.folder' and trashed=false"

            page_token = None
            while True:
                results = service.files().list(
                    q=subfolder_query,
                    pageSize=100,
                    fields="nextPageToken, files(id, name)",
                    pageToken=page_token
                ).execute()

                subfolders = results.get('files', [])

                for subfolder in subfolders:
                    subfolder_name = f"{folder_name}/{subfolder['name']}"
                    subdocs = list_docs_in_folder(
                        service,
                        subfolder['id'],
                        subfolder_name,
                        include_subfolders=True
                    )
                    all_docs.extend(subdocs)

                page_token = results.get('nextPageToken')
                if not page_token:
                    break

    except HttpError as error:
        print(f"[ERROR] Failed to list documents in '{folder_name}': {error}")

    return all_docs


def download_doc(service, doc_id, doc_name, folder_name):
    """
    Download a Google Doc as plain text and format as markdown

    Args:
        service: Google Drive API service object
        doc_id: Document ID to download
        doc_name: Document name
        folder_name: Source folder name

    Returns:
        Formatted markdown string, or None if download fails
    """
    try:
        # Export as plain text
        content = service.files().export(
            fileId=doc_id,
            mimeType='text/plain'
        ).execute()

        # Decode bytes to string
        text_content = content.decode('utf-8')

        # Get current timestamp
        timestamp = datetime.datetime.now().strftime('%A, %B %d, %Y at %I:%M %p')

        # Format as markdown with metadata
        markdown = f"# {doc_name}\n\n"
        markdown += f"*Source: Google Docs - {folder_name}*\n"
        markdown += f"*Last synced: {timestamp}*\n\n"
        markdown += "---\n\n"
        markdown += text_content

        return markdown

    except HttpError as error:
        print(f"[ERROR] Failed to download '{doc_name}': {error}")
        return None


def download_pdf(service, doc_id, doc_name, folder_name):
    """
    Download a PDF and extract text content, format as markdown

    Args:
        service: Google Drive API service object
        doc_id: PDF file ID to download
        doc_name: PDF file name
        folder_name: Source folder name

    Returns:
        Formatted markdown string with extracted text, or None if download/extraction fails
    """
    if not PDF_SUPPORT:
        print(f"[ERROR] PDF support not available. Install pypdf: pip install pypdf")
        return None

    try:
        # Download PDF file
        request = service.files().get_media(fileId=doc_id)
        pdf_buffer = io.BytesIO()
        downloader = MediaIoBaseDownload(pdf_buffer, request)

        done = False
        while not done:
            status, done = downloader.next_chunk()

        # Extract text from PDF
        pdf_buffer.seek(0)
        pdf_reader = PdfReader(pdf_buffer)

        # Get current timestamp
        timestamp = datetime.datetime.now().strftime('%A, %B %d, %Y at %I:%M %p')

        # Format as markdown with metadata
        markdown = f"# {doc_name}\n\n"
        markdown += f"*Source: PDF - {folder_name}*\n"
        markdown += f"*Last synced: {timestamp}*\n"
        markdown += f"*Pages: {len(pdf_reader.pages)}*\n\n"
        markdown += "---\n\n"

        # Extract text from each page
        for page_num, page in enumerate(pdf_reader.pages, start=1):
            page_text = page.extract_text()
            if page_text.strip():
                markdown += f"## Page {page_num}\n\n"
                markdown += page_text + "\n\n"

        return markdown

    except HttpError as error:
        print(f"[ERROR] Failed to download PDF '{doc_name}': {error}")
        return None
    except Exception as error:
        print(f"[ERROR] Failed to extract text from PDF '{doc_name}': {error}")
        return None


def sync_documents(service, folder_id, folder_name, output_dir, include_subfolders=False):
    """
    Sync all documents from a specific folder to output directory

    Args:
        service: Google Drive API service object
        folder_id: Folder ID to sync
        folder_name: Human-readable folder name
        output_dir: Path object for output directory
        include_subfolders: Whether to recursively scan subfolders

    Returns:
        Tuple of (synced_count, error_count, synced_files_set)
    """
    # Ensure output directory exists
    output_dir.mkdir(parents=True, exist_ok=True)

    # Track synced files for cleanup
    synced_files = set()
    synced_count = 0
    error_count = 0

    # Collect all documents from folder
    all_docs = list_docs_in_folder(service, folder_id, folder_name, include_subfolders)

    print(f"\n[SYNC] Total documents to sync from '{folder_name}': {len(all_docs)}")

    # Download and save each document
    for doc in all_docs:
        doc_id = doc['id']
        doc_name = doc['name']
        doc_type = doc['mimeType']
        doc_folder = doc['folder_name']

        print(f"\n[DOWNLOAD] '{doc_name}' from '{doc_folder}'...")

        # Download based on mimeType
        markdown = None
        if doc_type == 'application/vnd.google-apps.document':
            markdown = download_doc(service, doc_id, doc_name, doc_folder)
        elif doc_type == 'application/pdf':
            # Strip .pdf extension from name before sanitizing
            name_without_ext = doc_name[:-4] if doc_name.endswith('.pdf') else doc_name
            markdown = download_pdf(service, doc_id, name_without_ext, doc_folder)
            # Use the name without extension for filename
            doc_name = name_without_ext

        if markdown:
            # Save to file
            filename = sanitize_filename(doc_name)
            output_path = output_dir / filename

            try:
                output_path.write_text(markdown, encoding='utf-8')
                print(f"[SAVE] ✓ Saved to {filename}")
                synced_files.add(filename)
                synced_count += 1
            except IOError as e:
                print(f"[ERROR] Failed to save '{filename}': {e}")
                error_count += 1
        else:
            error_count += 1

    return synced_count, error_count, synced_files


def cleanup_stale_files(output_dir, synced_files):
    """
    Remove local files that no longer exist in Google Drive

    Args:
        output_dir: Path object for output directory
        synced_files: Set of filenames that were synced successfully

    Returns:
        Number of files removed
    """
    if not output_dir.exists():
        return 0

    removed_count = 0

    print(f"\n[CLEANUP] Checking for stale files in {output_dir.relative_to(ROSIE_DIR)}...")

    for local_file in output_dir.glob('*.md'):
        filename = local_file.name

        if filename not in synced_files:
            try:
                local_file.unlink()
                print(f"[CLEANUP] ✗ Removed stale file: {filename}")
                removed_count += 1
            except OSError as e:
                print(f"[ERROR] Failed to remove '{filename}': {e}")

    if removed_count == 0:
        print("[CLEANUP] No stale files found")

    return removed_count


def main():
    """Main sync flow"""
    print("=" * 80)
    print("ROSIE GOOGLE DOCS SYNC")
    print("=" * 80)

    # Check PDF support
    if PDF_SUPPORT:
        print("[INFO] PDF support enabled")
    else:
        print("[WARNING] PDF support not available. Install pypdf for PDF extraction:")
        print("         pip install pypdf")

    # Load configuration
    config = load_config()
    if not config:
        return

    # Authenticate
    creds = authenticate()
    if not creds:
        return

    # Build Drive service
    try:
        service = build('drive', 'v3', credentials=creds)
    except Exception as e:
        print(f"[ERROR] Failed to build Drive service: {e}")
        return

    # Get folder configurations
    folders = config.get('folders', {})
    public_folder = folders.get('public', {})
    private_folder = folders.get('private', {})

    # Track total statistics
    total_synced = 0
    total_errors = 0
    total_removed = 0

    # Sync public folder
    if public_folder.get('enabled', True):
        print("\n" + "=" * 80)
        print("SYNCING PUBLIC FOLDER")
        print("=" * 80)

        folder_id = public_folder.get('id')
        folder_name = public_folder.get('name', 'Rosie_Knowledge/Public')
        include_subfolders = public_folder.get('include_subfolders', True)

        synced, errors, synced_files = sync_documents(
            service, folder_id, folder_name, DOCS_OUTPUT_DIR, include_subfolders
        )
        removed = cleanup_stale_files(DOCS_OUTPUT_DIR, synced_files)

        total_synced += synced
        total_errors += errors
        total_removed += removed

        print(f"\nPublic folder: {synced} synced, {errors} errors, {removed} removed")
    else:
        print("\n[SKIP] Public folder is disabled")

    # Sync private folder
    if private_folder.get('enabled', True):
        print("\n" + "=" * 80)
        print("SYNCING PRIVATE FOLDER")
        print("=" * 80)

        folder_id = private_folder.get('id')
        folder_name = private_folder.get('name', 'Rosie_Knowledge/Private')
        include_subfolders = private_folder.get('include_subfolders', True)

        synced, errors, synced_files = sync_documents(
            service, folder_id, folder_name, PRIVATE_DOCS_OUTPUT_DIR, include_subfolders
        )
        removed = cleanup_stale_files(PRIVATE_DOCS_OUTPUT_DIR, synced_files)

        total_synced += synced
        total_errors += errors
        total_removed += removed

        print(f"\nPrivate folder: {synced} synced, {errors} errors, {removed} removed")
    else:
        print("\n[SKIP] Private folder is disabled")

    # Print summary
    print("\n" + "=" * 80)
    print("SYNC COMPLETE!")
    print("=" * 80)
    print(f"\nSummary:")
    print(f"  ✓ Total documents synced: {total_synced}")
    print(f"  ✗ Total errors: {total_errors}")
    print(f"  ✗ Total stale files removed: {total_removed}")
    print(f"\nOutput directories:")
    print(f"  Public:  {DOCS_OUTPUT_DIR.relative_to(ROSIE_DIR)}")
    print(f"  Private: {PRIVATE_DOCS_OUTPUT_DIR.relative_to(ROSIE_DIR)}")
    print(f"\nGoogle Docs and PDFs are now available in ROSIE's knowledge base.")
    print(f"ROSIE can answer questions based on your synced documents.")
    print("\n" + "=" * 80)


if __name__ == '__main__':
    main()
