#!/usr/bin/with-contenv bashio

# Read configuration from Home Assistant add-on options
export B4M_API_KEY=$(bashio::config 'b4m_api_key')
export HA_B4M_SESSION_ID=$(bashio::config 'ha_b4m_session_id')
export B4M_USER_ID=$(bashio::config 'b4m_user_id')
export SHIM_API_KEY=$(bashio::config 'shim_api_key')
export SESSION_TTL_SEC=$(bashio::config 'session_ttl_sec')
export MAX_TURNS=$(bashio::config 'max_turns')
export TIMEOUT_MS=$(bashio::config 'timeout_ms')
export POLL_INTERVAL_MS=$(bashio::config 'poll_interval_ms')
export POLL_MAX_INTERVAL_MS=$(bashio::config 'poll_max_interval_ms')
export B4M_BASE=$(bashio::config 'b4m_base')
export HA_TOOL_FUNCTION_NAME=$(bashio::config 'ha_tool_function_name')

# Validate required configuration
if [ -z "$B4M_API_KEY" ]; then
    bashio::log.fatal "b4m_api_key is required"
    exit 1
fi

if [ -z "$HA_B4M_SESSION_ID" ]; then
    bashio::log.fatal "ha_b4m_session_id is required"
    exit 1
fi

if [ -z "$B4M_USER_ID" ]; then
    bashio::log.fatal "b4m_user_id is required"
    exit 1
fi

bashio::log.info "Starting bike4mind OpenAI Shim..."

# Run the Python application
exec python3 /app/app.py
