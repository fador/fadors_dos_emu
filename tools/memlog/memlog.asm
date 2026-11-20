; MEMLOG.COM - resident INT 21h hook that appends DOS memory calls
; (AH=48h/49h/4Ah/58h) and, after AH=48h or a failed AH=4Ah, the MCB
; chain to C:\MEMLOG.TXT.  Reference tool for comparing DOS behaviour.
        org 100h
start:  jmp install

old21   dd 0
fname   db 'C:\MEMLOG.TXT',0
tsrpsp  dw 0
userpsp dw 0
inAX    dw 0
inBX    dw 0
inES    dw 0
outAX   dw 0
outBX   dw 0
outFL   dw 0
busy    db 0
handle  dw 0
buf     times 96 db 0

callold:                        ; INT 21h through the saved vector
        pushf
        call far [cs:old21]
        ret

hook:   cmp byte [cs:busy],0
        jne .pass
        cmp ah,48h
        je .log
        cmp ah,49h
        je .log
        cmp ah,4Ah
        je .log
        cmp ah,58h
        je .log
.pass:  jmp far [cs:old21]
.log:   mov [cs:inAX],ax
        mov [cs:inBX],bx
        mov [cs:inES],es
        pushf
        call far [cs:old21]
        pushf
        push ax
        push bx
        push cx
        push dx
        push si
        push di
        push ds
        push es
        push bp
        mov [cs:outAX],ax
        mov [cs:outBX],bx
        pushf
        pop word [cs:outFL]
        push cs
        pop ds
        push cs
        pop es
        mov byte [busy],1
        call openlog
        ; "AX=xxxx BX=xxxx ES=xxxx -> AX=xxxx BX=xxxx CF=x"
        mov di,buf
        mov si,s_in
        call puts
        mov ax,[inAX]
        call hex16
        mov si,s_bx
        call puts
        mov ax,[inBX]
        call hex16
        mov si,s_es
        call puts
        mov ax,[inES]
        call hex16
        mov si,s_out
        call puts
        mov ax,[outAX]
        call hex16
        mov si,s_bx
        call puts
        mov ax,[outBX]
        call hex16
        mov si,s_cf
        call puts
        mov al,[outFL]
        and al,1
        add al,'0'
        stosb
        call crlf
        call flushbuf
        mov ax,[inAX]
        cmp ah,48h
        je .chain
        cmp ah,4Ah
        jne .done
        test byte [outFL],1
        jz .done
.chain: call dumpchain
.done:  call closelog
        mov byte [busy],0
        pop bp
        pop es
        pop ds
        pop di
        pop si
        pop dx
        pop cx
        pop bx
        pop ax
        popf
        retf 2

s_in    db 'AX=',0
s_bx    db ' BX=',0
s_es    db ' ES=',0
s_out   db ' -> AX=',0
s_cf    db ' CF=',0
s_mcb   db '  MCB ',0
s_own   db ' owner=',0
s_sz    db ' size=',0

puts:   lodsb
        or al,al
        jz .e
        stosb
        jmp puts
.e:     ret
crlf:   mov ax,0A0Dh
        stosw
        ret
hex16:  push ax
        mov al,ah
        call hex8
        pop ax
hex8:   push ax
        shr al,4
        call hex4
        pop ax
hex4:   and al,0Fh
        add al,'0'
        cmp al,'9'
        jbe .s
        add al,7
.s:     stosb
        ret

openlog:                        ; switch to our PSP, open for append
        mov ah,51h
        call callold
        mov [userpsp],bx
        mov ah,50h
        mov bx,[tsrpsp]
        call callold
        mov ax,3D01h
        mov dx,fname
        call callold
        jnc .ok
        mov ah,3Ch
        xor cx,cx
        mov dx,fname
        call callold
.ok:    mov [handle],ax
        mov bx,ax
        mov ax,4202h
        xor cx,cx
        xor dx,dx
        call callold
        ret
closelog:
        mov ah,3Eh
        mov bx,[handle]
        call callold
        mov ah,50h
        mov bx,[userpsp]
        call callold
        ret
flushbuf:                       ; write buf..di
        mov cx,di
        sub cx,buf
        mov ah,40h
        mov bx,[handle]
        mov dx,buf
        call callold
        ret
dumpchain:
        mov ah,52h
        call callold            ; ES:BX = list of lists
        mov ax,[es:bx-2]
        push cs
        pop es
        mov [mcbseg],ax
        mov bp,300
.next:  push ds
        mov ds,[cs:mcbseg]
        mov al,[0]
        mov [cs:mcbtype],al
        mov ax,[1]
        mov [cs:mcbown],ax
        mov ax,[3]
        mov [cs:mcbsize],ax
        pop ds
        mov di,buf
        mov si,s_mcb
        call puts
        mov ax,[mcbseg]
        call hex16
        mov al,' '
        stosb
        mov al,[mcbtype]
        stosb
        mov si,s_own
        call puts
        mov ax,[mcbown]
        call hex16
        mov si,s_sz
        call puts
        mov ax,[mcbsize]
        call hex16
        call crlf
        call flushbuf
        cmp byte [mcbtype],'Z'
        je .end
        mov ax,[mcbseg]
        add ax,[mcbsize]
        inc ax
        mov [mcbseg],ax
        dec bp
        jnz .next
.end:   ret

mcbseg  dw 0
mcbown  dw 0
mcbsize dw 0
mcbtype db 0

install:
        mov [tsrpsp],cs
        mov ax,3521h
        int 21h
        mov [old21],bx
        mov [old21+2],es
        mov ah,3Ch              ; truncate the log
        xor cx,cx
        mov dx,fname
        int 21h
        jc .nofile
        mov bx,ax
        mov ah,3Eh
        int 21h
.nofile:
        mov ax,2521h
        mov dx,hook
        int 21h
        mov dx,(install-start+100h+15)/16
        mov ax,3100h
        int 21h
