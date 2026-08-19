#[macro_export] macro_rules! define_hardware {
    (
        $main_struct:ident {
            // Match all grouped subsystems (e.g., I2cConfig, SdConfig)
            $(
                $group_field:ident : $group_struct:ident {
                    $( $sub_field:ident : $sub_pin:ident ),* $(,)?
                }
            ),* $(,)?
        }
        // Servos get their own section because a PWM slice drives exactly two
        // of them, and which one is channel A vs B is fixed by the GPIO number
        // (even = A, odd = B). Naming the `a:`/`b:` slots here is the only
        // place that fact appears — `output::servo` never sees a slice or a
        // channel. Putting a channel-B pin in an `a:` slot is a compile error,
        // because `Pwm::new_output_ab` demands `ChannelAPin`/`ChannelBPin`.
        //
        // Reads as: `<field on the hardware struct>: <peripherals> -> <outputs>`.
        servos $servo_field:ident : $servo_struct:ident -> $servo_outputs:ident {
            $(
                $slice_field:ident : $slice:ident {
                    a: $a_field:ident = $a_pin:ident,
                    b: $b_field:ident = $b_pin:ident $(,)?
                }
            ),* $(,)?
        }
    ) => {
        // 1. Generate all the sub-structs
        $(
            pub struct $group_struct {
                $( pub $sub_field: embassy_rp::Peri<'static, embassy_rp::peripherals::$sub_pin> ),*
            }
        )*

        // 2. Generate the servo peripherals struct: one slice plus its two pins
        //    per declared pair.
        pub struct $servo_struct {
            $(
                pub $slice_field: embassy_rp::Peri<'static, embassy_rp::peripherals::$slice>,
                pub $a_field: embassy_rp::Peri<'static, embassy_rp::peripherals::$a_pin>,
                pub $b_field: embassy_rp::Peri<'static, embassy_rp::peripherals::$b_pin>,
            )*
        }

        // 3. Generate the per-fin trim table. Same field names as the outputs,
        //    so the compiler catches a fin that gains a pin but no trim. The
        //    values themselves live with the other calibration constants in
        //    `config`, not in this wiring declaration — re-trimming on the
        //    bench should not touch pin assignments.
        #[derive(Debug, Clone, Copy)]
        pub struct Trims {
            $(
                pub $a_field: crate::math::Deg,
                pub $b_field: crate::math::Deg,
            )*
        }

        // 4. Generate the live servo outputs. Each field is an independently
        //    controllable fin — writing one never disturbs its slice-mate,
        //    which `Pwm`'s own `SetDutyCycle` impl cannot promise (it writes
        //    both compare registers at once).
        pub struct $servo_outputs {
            $(
                pub $a_field: crate::output::servo::Fin,
                pub $b_field: crate::output::servo::Fin,
            )*
        }

        impl $servo_struct {
            /// Claim every servo pin, configuring one PWM slice per pair and
            /// splitting it into per-fin channels. `config` applies to every
            /// slice — `top` and `divider` are per-slice registers, so all
            /// servos necessarily share a frame rate.
            pub fn into_servos(
                self,
                config: &embassy_rp::pwm::Config,
                trims: Trims,
            ) -> $servo_outputs {
                $(
                    // `new_output_ab` populates both channels, so `split()`
                    // can only yield `(Some, Some)` here — but that is a fact
                    // about the call above, not something the type carries.
                    let (Some($a_field), Some($b_field)) = embassy_rp::pwm::Pwm::new_output_ab(
                        self.$slice_field,
                        self.$a_field,
                        self.$b_field,
                        config.clone(),
                    )
                    .split() else {
                        unreachable!()
                    };
                )*
                $servo_outputs {
                    $(
                        $a_field: crate::output::servo::Fin::new($a_field, trims.$a_field),
                        $b_field: crate::output::servo::Fin::new($b_field, trims.$b_field),
                    )*
                }
            }
        }

        impl $servo_outputs {
            /// Every fin, in declaration order. For sweeps and for commands
            /// that apply to all fins at once; individual fins are just named
            /// fields.
            pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut crate::output::servo::Fin> {
                [ $( &mut self.$a_field, &mut self.$b_field, )* ].into_iter()
            }
        }

        // 5. Generate the main hardware struct
        pub struct $main_struct {
            $( pub $group_field: $group_struct, )*
            pub $servo_field: $servo_struct,
        }

        // 5. Generate the partial-move extraction macro
        #[macro_export]
        macro_rules! take_hardware {
            ($p:expr) => {
                crate::config::board::$main_struct {
                    $(
                        $group_field: crate::config::board::$group_struct {
                            $( $sub_field: $p.$sub_pin ),*
                        },
                    )*
                    $servo_field: crate::config::board::$servo_struct {
                        $(
                            $slice_field: $p.$slice,
                            $a_field: $p.$a_pin,
                            $b_field: $p.$b_pin,
                        )*
                    },
                }
            }
        }
    };
}
