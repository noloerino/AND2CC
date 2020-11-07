//! Definitions for Kobuki sensors and utilities.
//! See https://github.com/noloerino/149-labs/tree/master/software/libraries/kobuki

#![allow(dead_code)]
pub mod actuator;
pub mod sensors;

// This comes from the utilities file
// https://github.com/lab11/buckler/blob/master/software/libraries/kobuki/kobukiUtilities.c
fn checksum(buf: &[u8]) -> u8 {
    let mut cs: u8 = 0;
    for e in buf.iter().skip(2) {
        cs ^= e;
    }
    cs
}
