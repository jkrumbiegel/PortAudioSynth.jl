cd(@__DIR__)
using Pkg
Pkg.activate(".")

includet("PortAudioSynth.jl")
const PAS = PortAudioSynth
using .PortAudioSynth






##

a = AudioEngine(framecount = 64)

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

Threads.@spawn start!(a)
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

struct Synth
    voices::Vector{PeriodicGenerator}
    playing::Vector{Bool}
    keys::Vector{Int}
end

function PortAudioSynth.next_sample!(synth::Synth)
    s = 0f0
    for (voice, playing) in zip(synth.voices, synth.playing)
        if playing
            s += PortAudioSynth.next_sample!(voice)
        end
    end
    s
end

function handle_key!(s::Synth, keyindex, action)
    function key_to_freq(keyindex)
        440 * 2 ^ (keyindex / 12)
    end
    
    j = findfirst(==(keyindex), s.keys)
    if j !== nothing
        if action == SDL_KEYUP
            s.playing[j] = false
        elseif action == SDL_KEYDOWN
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
synth = Synth(
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
            SDL_SCANCODE_A,
            SDL_SCANCODE_W,
            SDL_SCANCODE_S,
            SDL_SCANCODE_E,
            SDL_SCANCODE_D,
            SDL_SCANCODE_F,
            SDL_SCANCODE_T,
            SDL_SCANCODE_G,
            SDL_SCANCODE_Y,
            SDL_SCANCODE_H,
            SDL_SCANCODE_U,
            SDL_SCANCODE_J,
            SDL_SCANCODE_K,
        ]
        Dict((possible_keys .=> 1:length(possible_keys))...)
    end
    get(d, key, nothing)
end

##

a = AudioEngine(framecount = 128, n_buffers = 6)
push!(a.tracks, track)

Threads.@spawn start!(a)


##
stop!(a)



##

using SimpleDirectMediaLayer
using SimpleDirectMediaLayer.LibSDL2

##
@assert SDL_Init(SDL_INIT_EVERYTHING) == 0 "error initializing SDL: $(unsafe_string(SDL_GetError()))"

win = SDL_CreateWindow(
    "Game",
    SDL_WINDOWPOS_CENTERED,
    SDL_WINDOWPOS_CENTERED,
    200,
    200,
    SDL_WINDOW_SHOWN
)
SDL_SetWindowResizable(win, SDL_TRUE)

@async try
    close = false
    while !close
        event_ref = Ref{SDL_Event}()
        while Bool(SDL_PollEvent(event_ref))
            evt = event_ref[]
            evt_ty = evt.type
            if evt_ty == SDL_QUIT
                close = true
                break
            elseif evt_ty in (SDL_KEYDOWN, SDL_KEYUP)
                scan_code = evt.key.keysym.scancode
                keyindex = get_keyindex(scan_code)
                keyindex === nothing && break
                handle_key!(synth, keyindex, evt_ty)
                break
            end
        end

        sleep(0.001)
    end
finally
    println("SDL Done")
    SDL_DestroyWindow(win)
    SDL_Quit()
end

##

timestamps = let
    timestamps = zeros(1000)
    i = 1
    t = Timer(0.0, interval = 0.0008) do t
        if i <= 1000
            timestamps[i] = time()
            i += 1
        end
    end
    sleep(1.2)
    close(t)
    timestamps |> diff
end