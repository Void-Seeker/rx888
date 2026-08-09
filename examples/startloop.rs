// Why does starting a stream intermittently fail? The C boundary flattens every
// failure to SDDC_ERROR_IO (-4), so run the Rust API directly and print what the
// USB layer actually said.
use sddc::Radio;
use std::time::{Duration, Instant};

fn main() {
    let iters: usize = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(40);
    let gap_ms: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(0);

    let mut radio = match Radio::open(0) {
        Ok(r) => r,
        Err(e) => { println!("open failed: {e:?}"); return; }
    };
    println!("opened; {iters} start/stop cycles, {gap_ms} ms between");

    let mut fails = 0usize;
    for i in 0..iters {
        let t0 = Instant::now();
        match radio.read_async(Box::new(|_d: Option<&[i16]>| {})) {
            Ok(()) => {
                std::thread::sleep(Duration::from_millis(60));
                if let Err(e) = radio.read_cancel() { println!("  {i:3}: cancel failed: {e:?}"); }
            }
            Err(e) => {
                fails += 1;
                println!("  {i:3}: START FAILED after {:?}: {e:?}", t0.elapsed());
                let _ = radio.read_cancel();
            }
        }
        if gap_ms > 0 { std::thread::sleep(Duration::from_millis(gap_ms)); }
    }
    println!("{fails} failures in {iters} cycles");
}
