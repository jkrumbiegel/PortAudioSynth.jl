cd(@__DIR__)
using Pkg
Pkg.activate(".")

includet("PortAudioSynth.jl")
const PAS = PortAudioSynth
using .PortAudioSynth

using RTMidi
using MIDI



##

a = AudioEngine()

ps = Tuple(PeriodicGenerator(PAS.triangle, 44100, 0.0, freq, 0.3, 0.0)
    for freq in 0.6 .* [100.0, 200.0, 300.0])
c = CombinedGenerator(
    ps,
)
g = Gain(1.0)
push!(a.tracks, Track(
    c,
    Effect[
        g,
        VolumeWobble(1.0, 44100, 0.0, 3, 0.0),
        VolumeWobble(1.0, 44100, 0.0, 1.5, 0.0),
        Delay(44100, 0.5, 0.5),
    ]
))
##
Threads.@spawn start!(a)
##

for i in 1:100
    for p in ps
        p.frequency_hz = p.frequency_hz + randn()
    end
    sleep(1/30)
end

##
g.gain = 1
g.gain = 0

##
stop!(a)

##







###

mutable struct ASDR
    attack::Float64
    sustain::Float64
    decay::Float64
    release::Float64
    gain::Float64
    pressed::Bool
    rising::Bool
    last_t::Union{Nothing, Float64}
end

ASDR(attack, sustain, decay, release) = ASDR(attack, sustain, decay, release, 0, false, true, nothing)

mutable struct Voice{T}
    generator::T
    playing::Bool
    gain::Float64
    asdr::ASDR
end

mutable struct Synth
    voices::Vector{Voice}
    key_voice_mapping::Dict{Int, Int}
    next_voice::Int
end

function PortAudioSynth.next_sample!(synth::Synth, t)::Float32
    s = 0f0
    for voice in synth.voices
        if voice.playing
            s += PortAudioSynth.next_sample!(voice, t)
        end
    end
    s
end

function update_asdr!(asdr::ASDR, t)
    if asdr.last_t === nothing
        asdr.last_t = t
    end
    dt = t - asdr.last_t
    if asdr.pressed
        if asdr.rising
            asdr.gain = min(1.0, asdr.gain + dt / asdr.attack)
            if asdr.gain == 1.0
                asdr.rising = false
            end
        else
            asdr.gain = max(asdr.sustain, asdr.gain - dt / asdr.decay)
        end
    else
        asdr.gain = max(0.0, asdr.gain - dt / asdr.release)
    end
    asdr.last_t = t
end

function PortAudioSynth.next_sample!(v::Voice, t)::Float32
    update_asdr!(v.asdr, t)
    s = PortAudioSynth.next_sample!(v.generator, t)
    s * v.asdr.gain * v.gain
end

note_to_frequency(note) = 440 * 2 ^ ((note - 69) / 12)

handle_event!(s::Synth, event) = nothing

function reset!(v::Voice)
    v.asdr.pressed = true
    v.asdr.rising = true
    v.asdr.gain = 0
    v.asdr.last_t = nothing
    return
end

function handle_event!(s::Synth, event::Union{MIDI.NoteOnEvent, MIDI.NoteOffEvent})
    if haskey(s.key_voice_mapping, event.note)
        i = s.key_voice_mapping[event.note]
        if event isa MIDI.NoteOffEvent
            s.voices[i].asdr.pressed = false
            delete!(s.key_voice_mapping, event.note)
        end
    else
        if event isa MIDI.NoteOnEvent
            j = s.next_voice
            s.next_voice = mod1(s.next_voice + 1, length(s.voices))
            reset!(s.voices[j])
            s.voices[j].generator.frequency_hz = note_to_frequency(event.note)
            s.key_voice_mapping[event.note] = j
            s.voices[j].playing = true
        end
    end
    return
end




##
synth = Synth(
    [
        Voice(
            PeriodicGenerator(PAS.triangle, 44100, 0.0, 0.0, 0.3, 0.0),
            false,
            1.0,
            ASDR(0.1, 0.5, 0.2, 1.2)
        )
        for i in 1:5
    ],
    Dict{Int, Int}(),
    1
)

track = Track(
    synth,
    Effect[
        # Delay(44100, 0.4, 0.5)
    ]    
)

a = AudioEngine()
push!(a.tracks, track)

Threads.@spawn start!(a)

##

mi = MidiIn("RTMidi")
open_virtual_port!(mi, "RTMidi")


##

listen!(mi) do event
    handle_event!(synth, event)
end

##
stop_listening!(mi)
##
stop!(a)
