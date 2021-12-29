cd(@__DIR__)
using Pkg
Pkg.activate(".")

includet("PortAudioSynth.jl")
const PAS = PortAudioSynth
using .PortAudioSynth






##

a = AudioEngine()

push!(a.tracks, Track(
    CombinedGenerator(
        PeriodicGenerator(PAS.triangle, 44100, 0.0, 55.0, 0.3, 0.0),
        PeriodicGenerator(PAS.triangle, 44100, 0.0, 165.5, 0.3, 0.0),
        PeriodicGenerator(PAS.triangle, 44100, 0.0, 330.0, 0.3, 0.0),
    ),
    Effect[
        VolumeWobble(0.5, 44100, 0.0, 5, 0.0),
    ]
))

@async start(a)
##
stop(a)

##







###


using GLMakie






let
    f = Figure()
    display(f)

    running = Observable(false)
    b = Button(f[1, 1], label = @lift($running ? "Stop" : "Start"), tellwidth = false)

    freqs = 220 .* (1.0, 1.25, 1.5)
    generators = [PortAudioSynth.PeriodicGenerator(triangle, 44100, 0.0, freq, 0.3, 0.0) for freq in freqs]

    lsg = labelslidergrid!(f, "Sine Generator " .* ["1", "2", "3"], Ref(120:440), formats = "{}Hz")
    f[0, 1] = lsg.layout

    for (i, s) in enumerate(lsg.sliders)
        set_close_to!(s, freqs[i])
        on(s.value) do val
            generators[i].frequency_hz = val
        end
    end

    on(b.clicks) do c
        running[] = !running[]
        if running[]
            @async PortAudioSynth.run() do
                if !running[]
                    return nothing
                end

                sum(PortAudioSynth.next_sample!(s) for s in generators)
            end
        end
    end
end


struct Synth2
    voices::Vector{PortAudioSynth.PeriodicGenerator}
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
    [PortAudioSynth.PeriodicGenerator(triangle, 44100, 0.0, 0.0, 0.3, 0.0) for i in 1:5],
    zeros(Bool, 5),
    fill(-999, 5),
)
chain = PortAudioSynth.Track(
    synth,
    PortAudioSynth.Effect[
        PortAudioSynth.VolumeWobble(0.5, 44100, 0.0, 3, 0.0),
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

running = Observable(true)

@async PortAudioSynth.run() do
    if !running[]
        return nothing
    end
    PortAudioSynth.next_sample!(chain)
end

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


