// Copyright (C) 2022 StatiXOS
// SPDX-License-Identifer: Apache-2.0

//! Implementation of vendor.lineage.powershare-V1 for Oplus devices.

mod powershare_oplus;

use powershare_oplus::OplusPowerShare;

use vendor_lineage_powershare::aidl::vendor::lineage::powershare::{
    IPowerShare::BnPowerShare
};

use vendor_lineage_powershare::binder::BinderFeatures;

static HAL_SERVICE_NAME: &str = "vendor.lineage.powershare.IPowerShare/default";

fn main() {
    let powershare_binder = BnPowerShare::new_binder(
        OplusPowerShare,
        BinderFeatures::default(),
    );
    binder::add_service(HAL_SERVICE_NAME, powershare_binder.as_binder())
        .expect("Failed to register service?");
    // Does not return - spawn or perform any work you mean to do before this call.
    binder::ProcessState::join_thread_pool()
}
