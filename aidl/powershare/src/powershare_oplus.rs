// Copyright (C) 2022 StatiXOS
// SPDX-License-Identifer: Apache-2.0

use vendor_lineage_powershare::aidl::vendor::lineage::powershare::{
    IPowerShare::IPowerShare
};
use vendor_lineage_powershare::binder::{
    Interface, Result as BinderResult, StatusCode
};

const TX_ENABLE_PATH: &str = "/proc/wireless/enable_tx";

// Static file functions
fn set(path: &str, value: String) -> Result<(), StatusCode> {
    std::fs::write(path, value).map_err(|_| StatusCode::PERMISSION_DENIED)
}

fn get(path: &str) -> Option<String> {
    std::fs::read_to_string(path).ok()
}

pub struct OplusPowerShare;

impl Interface for OplusPowerShare {}

impl IPowerShare for OplusPowerShare {
    fn isEnabled(&self) -> BinderResult<bool> {
        if let Some(ans) = get(TX_ENABLE_PATH) {
            Ok(ans == "enable")
        } else {
            Ok(false)
        }
    }

    fn setEnabled(&self, enabled: bool) -> BinderResult<bool> {
        set(TX_ENABLE_PATH, if enabled { "1".to_string() } else { "0".to_string() })?;
        self.isEnabled()
    }

    fn getMinBattery(&self) -> BinderResult<i32> {
        Ok(0i32)
    }

    fn setMinBattery(&self, _level: i32) -> BinderResult<i32> {
        Ok(0i32)
    }

}

