cd(@__DIR__)
using Pkg
Pkg.activate(".")

includet("PortAudioSynth.jl")
const PAS = PortAudioSynth
using .PortAudioSynth






##

a = AudioEngine()

ps = Tuple(PeriodicGenerator(PAS.triangle, 44100, 0.0, freq, 0.3, 0.0)
    for freq in [100.0, 200.0, 300.0])
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

@async start!(a)
##

for i in 1:100
    for p in ps
        p.frequency_hz = p.frequency_hz + 0.5 * randn()
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


using GLMakie



struct Synth2
    voices::Vector{PeriodicGenerator}
    playing::Vector{Bool}
    keys::Vector{Int}
end

function PortAudioSynth.next_sample!(synth::Synth2)
    s = 0f0
    for (voice, playing) in zip(synth.voices, synth.playing)
        if playing
            s += PortAudioSynth.next_sample!(voice)
        end
    end
    s
end

function handle_key!(s::Synth2, keyindex, action)

    function key_to_freq(keyindex)
        440 * 2 ^ (keyindex / 12)
    end
    
    j = findfirst(==(keyindex), s.keys)
    if j !== nothing
        if action === Makie.Keyboard.release
            s.playing[j] = false
        elseif action === Makie.Keyboard.press
            s.playing[j] = true
        end
    else
        k = findfirst(!, s.playing)
        if k === nothing
            return
        else
            s.keys[k] = keyindex
            s.voices[k].frequency_hz = key_to_freq(keyindex)
            s.playing[k] = true
        end
    end
    return
end

##
synth = Synth2(
    [PeriodicGenerator(PAS.triangle, 44100, 0.0, 0.0, 0.3, 0.0) for i in 1:5],
    zeros(Bool, 5),
    fill(-999, 5),
)

track = Track(
    synth,
    Effect[
        Delay(44100, 0.4, 0.5)
    ]    
)

function get_keyindex(key)
    d = begin
        possible_keys = [
            Makie.Keyboard.a,
            Makie.Keyboard.w,
            Makie.Keyboard.s,
            Makie.Keyboard.e,
            Makie.Keyboard.d,
            Makie.Keyboard.f,
            Makie.Keyboard.t,
            Makie.Keyboard.g,
            Makie.Keyboard.y,
            Makie.Keyboard.h,
            Makie.Keyboard.u,
            Makie.Keyboard.j,
            Makie.Keyboard.k
        ]
        Dict((possible_keys .=> 1:length(possible_keys))...)
    end
    get(d, key, nothing)
end

##

s = Scene(camera = campixel!)

isblack(i) = mod1(i, 12) ∈ (2, 4, 7, 9, 11)

polys = let
    lastblack = false
    lastx = 0
    Observable(map(1:14) do i
        black = isblack(i)
        xshift = 35
        lastblack = black
        lastx = lastx + xshift
        Rect(lastx, 100, 30, 150)
    end)
end
keycolor(i) = isblack(i) ? :black : :gray95
colors = Observable(keycolor.(1:14))
poly!(s, polys, color = colors, show_axis = false)

display(s)

##

a = AudioEngine(framecount = 256, n_buffers = 8)
push!(a.tracks, track)

@async start!(a)

on(s.events.keyboardbutton) do event
    event.action === Makie.Keyboard.repeat && return
    if event.key == Makie.Keyboard.escape
        running[] = false
    else
        keyindex = get_keyindex(event.key)
        keyindex === nothing && return
        handle_key!(synth, keyindex, event.action)
        if event.action === Makie.Keyboard.press
            colors[][keyindex] = :red
            notify(colors)
        elseif event.action === Makie.Keyboard.release
            colors[][keyindex] = keycolor(keyindex)
            notify(colors)
        end
    end
end

##

stop!(a)

